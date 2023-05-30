# This file is part of ts_atmcssimulator.
#
# Developed for the Vera Rubin Observatory Telescope and Site Systems.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from __future__ import annotations

__all__ = ["McsSimulator"]

import asyncio
import logging
import types
import typing

import jsonschema
import numpy as np
from lsst.ts import salobj, simactuators, tcpip, utils
from lsst.ts.idl.enums.ATMCS import AtMountState, M3ExitPort, M3State

from .enums import Ack, Axis, CommandKey, Event, MainAxes, Telemetry
from .mcs_csc import ATMCSCsc
from .mcs_server_simulator import McsServerSimulator
from .schemas.registry import registry

CMD_ITEMS_TO_IGNORE = frozenset({CommandKey.ID, CommandKey.VALUE})


class McsSimulator:
    """Simulate the ATMCS system."""

    def __init__(self, csc: ATMCSCsc) -> None:
        self.log = logging.getLogger(type(self).__name__)
        # TODO DM-38912 Send "port" via configuration.
        self.cmd_evt_server = McsServerSimulator(
            host=tcpip.LOCALHOST_IPV4,
            port=5000,
            log=self.log,
            dispatch_callback=self.cmd_evt_dispatch_callback,
            connect_callback=self.connect_callback,
            name="CmdEvtMcsServer",
        )
        # TODO DM-38912 Send "port" via configuration.
        self.telemetry_server = McsServerSimulator(
            host=tcpip.LOCALHOST_IPV4,
            port=6000,
            log=self.log,
            dispatch_callback=self.telemetry_dispatch_callback,
            name="TelemetryMcsServer",
        )
        self.csc = csc

        # Keep track of the sequence_id as commands come dripping in. The
        # sequence ID should raise monotonally without gaps. If a gap is seen,
        # a NOACK should be returned.
        self.last_sequence_id = 0

        # Dict of command: function.
        self.dispatch_dict: dict[str, typing.Callable] = {
            "setInstrumentPort": self.set_instrument_port,
            "startTracking": self.start_tracking,
            "stopTracking": self.stop_tracking,
            "trackTarget": self.track_target,
        }

        # Task that runs while the events_and_telemetry_loop runs.
        self._events_and_telemetry_task = utils.make_done_future()
        # Task that runs while axes are slewing to a halt from stopTracking.
        self._stop_tracking_task = utils.make_done_future()
        # Task that runs while axes are halting before being disabled.
        self._disable_all_drives_task = utils.make_done_future()
        # Timer to kill tracking if trackTarget doesn't arrive in time.
        self._kill_tracking_timer = utils.make_done_future()

        # Dict of M3ExitPort (the instrument port M3 points to): tuple of:
        # * index of self.m3_port_positions: the M3 position for this port
        # * M3State (state of M3 axis when pointing to this port)
        # * Rotator axis at this port, as an Axis enum,
        #   or None if this port has no rotator.
        self._port_info_dict = {
            M3ExitPort.NASMYTH1: (0, M3State.NASMYTH1, Axis.NA1),
            M3ExitPort.NASMYTH2: (1, M3State.NASMYTH2, Axis.NA2),
            M3ExitPort.PORT3: (2, M3State.PORT3, None),
        }
        # Name of minimum limit switch event for each axis.
        self._min_lim_names = (
            "elevationLimitSwitchLower",
            "azimuthLimitSwitchCW",
            "nasmyth1LimitSwitchCW",
            "nasmyth2LimitSwitchCW",
            "m3RotatorLimitSwitchCW",
        )
        # Name of maximum limit switch event for each axis.
        self._max_lim_names = (
            "elevationLimitSwitchUpper",
            "azimuthLimitSwitchCCW",
            "nasmyth1LimitSwitchCCW",
            "nasmyth2LimitSwitchCCW",
            "m3RotatorLimitSwitchCCW",
        )
        # Name of "in position" event for each axis,
        # excluding ``allAxesInPosition``.
        self._in_position_names = (
            "elevationInPosition",
            "azimuthInPosition",
            "nasmyth1RotatorInPosition",
            "nasmyth2RotatorInPosition",
            "m3InPosition",
        )
        # Name of drive status events for each axis.
        self._drive_status_names = (
            ("elevationDriveStatus",),
            (
                "azimuthDrive1Status",
                "azimuthDrive2Status",
            ),
            ("nasmyth1DriveStatus",),
            ("nasmyth2DriveStatus",),
            ("m3DriveStatus",),
        )
        # Name of brake events for each axis.
        self._brake_names = (
            ("elevationBrake",),
            (
                "azimuthBrake1",
                "azimuthBrake2",
            ),
            ("nasmyth1Brake",),
            ("nasmyth2Brake",),
            (),
        )
        # Has tracking been enabled by startTracking?
        # This remains true until stopTracking is called or the
        # summary state is no longer salobj.State.Enabled,
        # even if some drives have been disabled by running into limits.
        self._tracking_enabled = False
        # Is this particular axis enabled?
        # This remains true until stopTracking is called or the
        # summary state is no longer salobj.State.Enabled,
        # or the axis runs into a limit.
        # Note that the brakes automatically come on/off
        # if the axis is disabled/enabled, respectively.
        self._axis_enabled = np.zeros([5], dtype=bool)
        # Timer to kill tracking if trackTarget doesn't arrive in time.

        # TODO DM-38912 Make this configurable.
        # Interval between telemetry updates (sec).
        self._telemetry_interval = 1
        # Number of event updates per telemetry update.
        self._events_per_telemetry = 10

        # TODO DM-38912 Make this configurable.
        # Configuration items.
        self.max_tracking_interval = np.full(5, 0.0)
        self.min_commanded_position = np.full(5, 0.0)
        self.max_commanded_position = np.full(5, 0.0)
        self.min_limit_switch_position = np.full(5, 0.0)
        self.max_limit_switch_position = np.full(5, 0.0)
        self.max_velocity = np.full(5, 0.0)
        self.topple_azimuth = np.full(5, 0.0)
        self.m3_port_positions = np.full(5, 0.0)
        self.axis_encoder_counts_per_deg = np.full(5, 0.0)
        self.motor_encoder_counts_per_deg = np.full(5, 0.0)
        self.motor_axis_ratio = np.full(5, 0.0)
        self.torque_per_accel = np.full(5, 0.0)
        self.nsettle = 0
        self.limit_overtravel = 0
        self.actuators: list[simactuators.TrackingActuator] = []
        # allowed position error for M3 to be considered in position (deg)
        self.m3tolerance = 1e-5

    # TODO DM-38912 Make this configurable.
    async def configure(
        self,
        max_tracking_interval: float = 2.5,
        min_commanded_position: np.ndarray = np.array(
            [5, -270, -165, -165, 0], dtype=float
        ),
        max_commanded_position: np.ndarray = np.array(
            [90, 270, 165, 165, 180], dtype=float
        ),
        start_position: np.ndarray = np.array([80, 0, 0, 0, 0], dtype=float),
        min_limit_switch_position: np.ndarray = np.array(
            [3, -272, -167, -167, -2], dtype=float
        ),
        max_limit_switch_position: np.ndarray = np.array(
            [92, 272, 167, 167, 182], dtype=float
        ),
        max_velocity: np.ndarray = np.array([5, 5, 5, 5, 5], dtype=float),
        max_acceleration: np.ndarray = np.array([3, 3, 3, 3, 3], dtype=float),
        topple_azimuth: np.ndarray = np.array([2, 5], dtype=float),
        m3_port_positions: np.ndarray = np.array([0, 180, 90], dtype=float),
        axis_encoder_counts_per_deg: np.ndarray = np.array(
            [3.6e6, 3.6e6, 3.6e6, 3.6e6, 3.6e6], dtype=float
        ),
        motor_encoder_counts_per_deg: np.ndarray = np.array(
            [3.6e5, 3.6e5, 3.6e5, 3.6e5, 3.6e5], dtype=float
        ),
        motor_axis_ratio: np.ndarray = np.array([100, 100, 100, 100, 100], dtype=float),
        torque_per_accel: np.ndarray = np.array([1, 1, 1, 1, 1], dtype=float),
        nsettle: int = 2,
        limit_overtravel: int = 1,
    ) -> None:
        """Set configuration.

        Parameters
        ----------
        max_tracking_interval : `float`
            Maximum time between tracking updates (sec)
        min_commanded_position : ``iterable`` of 5 `float`
            Minimum commanded position for each axis, in deg
        start_position :  : ``iterable`` of 5 `float`
            Initial position for each axis, in deg
        max_commanded_position : ``iterable`` of 5 `float`
            Minimum commanded position for each axis, in deg
        min_limit_switch_position : ``iterable`` of 5 `float`
            Position of minimum L1 limit switch for each axis, in deg
        max_limit_switch_position : ``iterable`` of 5 `float`
            Position of maximum L1 limit switch for each axis, in deg
        max_velocity : ``iterable`` of 5 `float`
            Maximum velocity of each axis, in deg/sec
        max_acceleration : ``iterable`` of 5 `float`
            Maximum acceleration of each axis, in deg/sec
        topple_azimuth : ``iterable`` of 2 `float`
            Min, max azimuth at which the topple block moves, in deg
        m3_port_positions : ``iterable`` of 3 `float`
            M3 position of instrument ports NA1, NA2 and Port3,
            in that order.
        axis_encoder_counts_per_deg : `list` [`float`]
            Axis encoder resolution, for each axis, in counts/deg
        motor_encoder_counts_per_deg : `list` [`float`]
            Motor encoder resolution, for each axis, in counts/deg
        motor_axis_ratio : `list` [`float`]
            Number of turns of the motor for one turn of the axis.
        torque_per_accel :  `list` [`float`]
            Motor torque per unit of acceleration,
            in units of measuredTorque/(deg/sec^2)
        nsettle : `int`
            Number of consecutive trackPosition commands that result in
            a tracking path before we report an axis is tracking.
        limit_overtravel : `float`
            Distance from limit switches to hard stops (deg).

        Raises
        ------
        lsst.ts.salobj.ExpectedError
            If any list argument has the wrong number of values,
            or if any value cannot be cast to float.
        lsst.ts.salobj.ExpectedError
            If any max_velocity or max_acceleration value <= 0.
        lsst.ts.salobj.ExpectedError
            If min_commanded_position > max_commanded_position
            or start_position > min_commanded_position
            or start_position > max_commanded_position for any axis.
        lsst.ts.salobj.ExpectedError
            If limit_overtravel < 0.
        """

        for axis in Axis:
            if max_commanded_position[axis] < min_commanded_position[axis]:
                raise salobj.ExpectedError(
                    f"max_commanded_position[{axis}]={max_commanded_position[axis]} <= "
                    f"min_commanded_position[{axis}]={min_commanded_position[axis]}"
                )
            if min_commanded_position[axis] > start_position[axis]:
                raise salobj.ExpectedError(
                    f"min_commanded_position[{axis}]={min_commanded_position[axis]} > "
                    f"start_position[{axis}]={start_position[axis]}"
                )
            if max_commanded_position[axis] < start_position[axis]:
                raise salobj.ExpectedError(
                    f"max_commanded_position[{axis}]={max_commanded_position[axis]} < "
                    f"start_position[{axis}]={start_position[axis]}"
                )

        if max_velocity.min() <= 0:
            raise salobj.ExpectedError(
                f"max_velocity={max_velocity}; all values must be positive"
            )
        if max_acceleration.min() <= 0:
            raise salobj.ExpectedError(
                f"max_acceleration={max_acceleration}; all values must be positive"
            )
        if limit_overtravel < 0:
            raise salobj.ExpectedError(
                f"limit_overtravel={limit_overtravel} must be >= 0"
            )

        self.max_tracking_interval = max_tracking_interval
        self.min_commanded_position = min_commanded_position
        self.max_commanded_position = max_commanded_position
        self.min_limit_switch_position = min_limit_switch_position
        self.max_limit_switch_position = max_limit_switch_position
        self.max_velocity = max_velocity
        self.topple_azimuth = topple_azimuth
        self.m3_port_positions = m3_port_positions
        self.axis_encoder_counts_per_deg = axis_encoder_counts_per_deg
        self.motor_encoder_counts_per_deg = motor_encoder_counts_per_deg
        self.motor_axis_ratio = motor_axis_ratio
        self.torque_per_accel = torque_per_accel
        self.nsettle = nsettle
        self.limit_overtravel = limit_overtravel

        tai = utils.current_tai()
        self.actuators = [
            simactuators.TrackingActuator(
                min_position=self.min_commanded_position[axis],
                max_position=self.max_commanded_position[axis],
                max_velocity=max_velocity[axis],
                max_acceleration=max_acceleration[axis],
                # Use 0 for M3 to prevent tracking.
                dtmax_track=0 if axis == 4 else self.max_tracking_interval,
                nsettle=self.nsettle,
                tai=tai,
                start_position=start_position[axis],
            )
            for axis in Axis
        ]
        self.actuators[0].verbose = True

    async def connect_callback(self, server: tcpip.OneClientServer) -> None:
        if server.connected:
            await self.write_evt(
                evt_id=Event.POSITIONLIMITS,
                minimum=self.min_commanded_position.tolist(),
                maximum=self.max_commanded_position.tolist(),
            )

            axes_to_enable: set[int] = {Axis.Elevation, Axis.Azimuth}
            tai = utils.current_tai()
            rot_axis = self.m3_port_rot(tai)[1]
            if rot_axis is not None:
                axes_to_enable.add(rot_axis)
            for axis in Axis:
                self._axis_enabled[axis] = axis in axes_to_enable

            await self.start_tasks()
        else:
            await self.stop_tasks()

    async def cmd_evt_dispatch_callback(self, data: dict[str, typing.Any]) -> None:
        data_ok = await self.verify_data(data=data)
        if not data_ok:
            await self.write_noack_response(sequence_id=data[CommandKey.SEQUENCE_ID])
            return

        await self.write_ack_response(sequence_id=data[CommandKey.SEQUENCE_ID])

        cmd = data[CommandKey.ID].replace("cmd_", "")
        func = self.dispatch_dict[cmd]
        kwargs = {
            key: value for key, value in data.items() if key not in CMD_ITEMS_TO_IGNORE
        }
        await func(**kwargs)

    async def telemetry_dispatch_callback(self, data: typing.Any) -> None:
        pass

    async def verify_data(self, data: dict[str, typing.Any]) -> bool:
        """Verify the format and values of the data.

        The format of the data is described at
        https://github.com/lsst-ts/ts_labview_tcp_json
        as well as in the JSON schemas in the schemas directory.

        Parameters
        ----------
        data : `dict` of `any`
            The dict to be verified.

        Returns
        -------
        bool:
            Whether the data follows the correct format and has the correct
            contents or not.
        """
        if CommandKey.ID not in data or CommandKey.SEQUENCE_ID not in data:
            self.log.error(f"Received invalid {data=}. Ignoring.")
            return False
        payload_id = data[CommandKey.ID].replace("cmd_", "command_")
        if payload_id not in registry:
            self.log.error(f"Unknown command in {data=}.")
            return False

        sequence_id = data[CommandKey.SEQUENCE_ID]
        if self.last_sequence_id == 0:
            self.last_sequence_id = sequence_id
        else:
            if sequence_id - self.last_sequence_id != 1:
                return False

        json_schema = registry[payload_id]
        try:
            jsonschema.validate(data, json_schema)
        except jsonschema.ValidationError as e:
            self.log.exception("Validation failed.", e)
            return False
        return True

    async def set_instrument_port(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        if CommandKey.PORT not in kwargs or self._tracking_enabled:
            await self.write_fail_response(sequence_id=sequence_id)
            return
        port = kwargs[CommandKey.PORT]
        try:
            m3_port_positions_ind = self._port_info_dict[port][0]
        except KeyError:
            await self.write_fail_response(sequence_id=sequence_id)
            return
        try:
            m3_port_positions = self.m3_port_positions[m3_port_positions_ind]
        except KeyError:
            await self.write_fail_response(sequence_id=sequence_id)
            return
        await self.write_evt(evt_id=Event.M3PORTSELECTED, selected=port)
        m3actuator = self.actuators[Axis.M3]
        m3_in_position = self.m3_in_position(utils.current_tai())
        if m3actuator.target.position == m3_port_positions and m3_in_position:
            # already there; don't do anything
            pass
        else:
            self.actuators[Axis.M3].set_target(
                tai=utils.current_tai(), position=m3_port_positions, velocity=0
            )
            self._axis_enabled[Axis.NA1] = False
            self._axis_enabled[Axis.NA2] = False
            await self.update_events()
        await self.write_success_response(sequence_id=sequence_id)

    async def start_tracking(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        m3_in_position = self.m3_in_position(utils.current_tai())
        if not m3_in_position:
            await self.write_fail_response(sequence_id=sequence_id)
            return
        if not self._stop_tracking_task.done():
            await self.write_fail_response(sequence_id=sequence_id)
            return
        self._tracking_enabled = True
        await self.update_events()
        # TODO DM-39408 Not sure what to do here. The original code in the CSC
        #  set the CSC to FAULT after a certain period was passed but there is
        #  no way to do that with the TCP/IP infrastructure.
        # self._set_tracking_timer(restart=True)
        await self.write_success_response(sequence_id=sequence_id)

    async def stop_tracking(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        if not self._stop_tracking_task.done():
            await self.write_fail_response(sequence_id=sequence_id)
            return

        # TODO DM-39408 Not sure what to do here. The original code in the CSC
        #  set the CSC to FAULT after a certain period was passed but there is
        #  no way to do that with the TCP/IP infrastructure.
        # self._set_tracking_timer(restart=True)
        self._tracking_enabled = False
        for axis in MainAxes:
            self.actuators[axis].stop()
        self._stop_tracking_task.cancel()
        self._stop_tracking_task = asyncio.create_task(self._finish_stop_tracking())
        await self.update_events()
        await self.write_success_response(sequence_id=sequence_id)

    async def track_target(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        if not self._tracking_enabled:
            await self.write_fail_response(sequence_id=sequence_id)
            return
        try:
            position = np.array(
                [
                    kwargs["elevation"],
                    kwargs["azimuth"],
                    kwargs["nasmyth1RotatorAngle"],
                    kwargs["nasmyth2RotatorAngle"],
                ],
                dtype=float,
            )
            velocity = np.array(
                [
                    kwargs["elevationVelocity"],
                    kwargs["azimuthVelocity"],
                    kwargs["nasmyth1RotatorAngleVelocity"],
                    kwargs["nasmyth2RotatorAngleVelocity"],
                ],
                dtype=float,
            )
            dt = utils.current_tai() - kwargs["taiTime"]
            current_position = position + dt * velocity
            if np.any(current_position < self.min_commanded_position[0:4]) or np.any(
                current_position > self.max_commanded_position[0:4]
            ):
                await self.write_fail_response(sequence_id=sequence_id)
                return
            if np.any(np.abs(velocity) > self.max_velocity[0:4]):
                await self.write_fail_response(sequence_id=sequence_id)
                return
        except Exception:
            await self.write_fail_response(sequence_id=sequence_id)
            return

        for i in range(4):
            self.actuators[i].set_target(
                tai=kwargs["taiTime"], position=position[i], velocity=velocity[i]
            )

        target_fields = (
            CommandKey.AZIMUTH,
            CommandKey.AZIMUTH_VELOCITY,
            CommandKey.ELEVATION,
            CommandKey.ELEVATION_VELOCITY,
            CommandKey.NASMYTH1_ROTATOR_ANGLE,
            CommandKey.NASMYTH1_ROTATOR_ANGLE_VELOCITY,
            CommandKey.NASMYTH2_ROTATOR_ANGLE,
            CommandKey.NASMYTH2_ROTATOR_ANGLE_VELOCITY,
            CommandKey.TAI_TIME,
            CommandKey.TRACK_ID,
            CommandKey.TRACK_SYS,
            CommandKey.RA_DE_SYS,
        )
        evt_kwargs = {k: v for k, v in kwargs.items() if k in target_fields}
        await self.write_evt(evt_id=Event.TARGET, **evt_kwargs)
        self.csc.tel_mount_AzEl_Encoders.set(trackId=kwargs["trackId"])
        self.csc.tel_mount_Nasmyth_Encoders.set(trackId=kwargs["trackId"])

        # TODO DM-39408 Not sure what to do here. The original code in the CSC
        #  set the CSC to FAULT after a certain period was passed but there is
        #  no way to do that with the TCP/IP infrastructure.
        # self._set_tracking_timer(restart=True)
        await self.write_success_response(sequence_id=sequence_id)

    def m3_port_rot(self, tai: float) -> tuple[int | None, int | None]:
        """Return exit port and rotator axis.

        Parameters
        ----------
        tai : `float`
            Current time, TAI unix seconds.

        Returns
        -------
        port_rot : `tuple`
            Exit port and rotator axis, as a tuple:

            * exit port: an M3ExitPort enum value
            * rotator axis: the instrument rotator at this port,
              as an Axis enum value, or None if the port has no rotator.
        """
        if not self.m3_in_position(tai):
            return (None, None)
        target_position = self.actuators[Axis.M3].target.position
        for exit_port, (ind, _, rot_axis) in self._port_info_dict.items():
            if self.m3_port_positions[ind] == target_position:
                return (exit_port, rot_axis)
        return (None, None)

    def m3_in_position(self, tai: float) -> bool:
        """Is the M3 actuator in position?

        Parameters
        ----------
        tai : `float`
            Current time, TAI unix seconds.
        """
        m3actuator = self.actuators[Axis.M3]
        if m3actuator.kind(tai) != m3actuator.Kind.Stopped:
            return False
        m3target_position = m3actuator.target.position
        m3current = m3actuator.path[-1].at(tai)
        m3position_difference = abs(m3target_position - m3current.position)
        return m3position_difference < self.m3tolerance

    async def update_events(self) -> None:
        """Update most events and output those that have changed.

        Notes
        -----
        Updates state of all non-generic events, except as noted:

        * ``atMountState``
        * ``m3State``
        * (``m3PortSelected`` is output by ``do_setInstrumentPort``)

        * ``elevationInPosition``
        * ``azimuthInPosition``
        * ``nasmyth1RotatorInPosition``
        * ``nasmyth2RotatorInPosition``
        * ``m3InPosition``
        * ``allAxesInPosition``

        * ``azimuthToppleBlockCCW``
        * ``azimuthToppleBlockCW``
        * ``azimuthLimitSwitchCW``
        * ``m3RotatorDetentLimitSwitch``

        * ``elevationLimitSwitchLower``
        * ``elevationLimitSwitchUpper``
        * ``azimuthLimitSwitchCCW``
        * ``nasmyth1LimitSwitchCCW``
        * ``nasmyth1LimitSwitchCW``
        * ``nasmyth2LimitSwitchCCW``
        * ``nasmyth2LimitSwitchCW``
        * ``m3RotatorLimitSwitchCCW``
        * ``m3RotatorLimitSwitchCW``

        * ``azimuthDrive1Status``
        * ``azimuthDrive2Status``
        * ``elevationDriveStatus``
        * ``nasmyth1DriveStatus``
        * ``nasmyth2DriveStatus``
        * ``m3DriveStatus``

        * ``elevationBrake``
        * ``azimuthBrake1``
        * ``azimuthBrake2``
        * ``nasmyth1Brake``
        * ``nasmyth2Brake``

        Report events that have changed,
        and for axes that have run into a limit switch, abort the axis,
        disable its drives and set its brakes.
        """
        try:
            tai = utils.current_tai()
            current_position = np.array(
                [actuator.path.at(tai).position for actuator in self.actuators],
                dtype=float,
            )
            m3actuator = self.actuators[Axis.M3]
            axes_in_use: set[int] = {Axis.Elevation, Axis.Azimuth, Axis.M3}

            # Handle M3 actuator; set_target needs to be called to transition
            # from slewing to tracking, and that is done here for M3
            # (the trackPosition command does that for the other axes).
            m3arrived = (
                m3actuator.kind(tai) == m3actuator.Kind.Slewing
                and tai > m3actuator.path[-1].tai
            )
            if m3arrived:
                segment = simactuators.path.PathSegment(
                    tai=tai, position=m3actuator.target.position
                )
                m3actuator.path = simactuators.path.Path(
                    segment, kind=m3actuator.Kind.Stopped
                )
            exit_port, rot_axis = self.m3_port_rot(tai)
            if rot_axis is not None:
                axes_in_use.add(rot_axis)
                if m3arrived:
                    self._axis_enabled[rot_axis] = True

            # Handle limit switches
            # including aborting axes that are out of limits
            # and putting on their brakes (if any)
            abort_axes = []
            for axis in Axis:
                await self.write_evt(
                    evt_id=self._min_lim_names[axis],
                    active=bool(
                        current_position[axis] < self.min_limit_switch_position[axis]
                    ),
                )
                await self.write_evt(
                    evt_id=self._max_lim_names[axis],
                    active=bool(
                        current_position[axis] > self.max_limit_switch_position[axis]
                    ),
                )
                if (
                    current_position[axis] < self.min_limit_switch_position[axis]
                    or current_position[axis] > self.max_limit_switch_position[axis]
                ):
                    abort_axes.append(axis)
            for axis in abort_axes:
                position = current_position[axis]
                position = max(
                    position,
                    self.min_limit_switch_position[axis] - self.limit_overtravel,
                )
                position = min(
                    position,
                    self.max_limit_switch_position[axis] + self.limit_overtravel,
                )
                self.actuators[axis].abort(tai=tai, position=position)
                self._axis_enabled[axis] = False

            # Handle brakes
            for axis in Axis:
                for brake_name in self._brake_names[axis]:
                    await self.write_evt(
                        evt_id=brake_name, engaged=bool(not self._axis_enabled[axis])
                    )

            # Handle drive status (which means enabled)
            for axis in Axis:
                for evt_name in self._drive_status_names[axis]:
                    await self.write_evt(
                        evt_id=evt_name, enable=bool(self._axis_enabled[axis])
                    )

            # Handle atMountState
            if self._tracking_enabled:
                mount_state = AtMountState.TRACKINGENABLED
            elif (
                not self._stop_tracking_task.done()
                or not self._disable_all_drives_task.done()
            ):
                mount_state = AtMountState.STOPPING
            else:
                mount_state = AtMountState.TRACKINGDISABLED
            await self.write_evt(evt_id=Event.ATMOUNTSTATE, state=mount_state)

            # Handle azimuth topple block
            if current_position[Axis.Azimuth] < self.topple_azimuth[0]:
                await self.write_evt(evt_id=Event.AZIMUTHTOPPLEBLOCKCCW, active=True)
                await self.write_evt(evt_id=Event.AZIMUTHTOPPLEBLOCKCW, active=False)
            elif current_position[Axis.Azimuth] > self.topple_azimuth[1]:
                await self.write_evt(evt_id=Event.AZIMUTHTOPPLEBLOCKCCW, active=False)
                await self.write_evt(evt_id=Event.AZIMUTHTOPPLEBLOCKCW, active=True)
            else:
                await self.write_evt(evt_id=Event.AZIMUTHTOPPLEBLOCKCCW, active=False)
                await self.write_evt(evt_id=Event.AZIMUTHTOPPLEBLOCKCW, active=False)

            # Handle m3InPosition
            # M3 is in position if the current velocity is 0
            # and the current position equals the commanded position.
            m3_in_position = self.m3_in_position(tai)
            await self.write_evt(evt_id=Event.M3INPOSITION, inPosition=m3_in_position)

            # Handle "in position" events for the main axes.
            # Main axes are in position if enabled
            # and actuator.kind(tai) is tracking.
            if not self._tracking_enabled:
                for axis in MainAxes:
                    await self.write_evt(
                        evt_id=self._in_position_names[axis], inPosition=False
                    )
                await self.write_evt(evt_id=Event.ALLAXESINPOSITION, inPosition=False)
            else:
                all_in_position = m3_in_position
                for axis in MainAxes:
                    if not self._axis_enabled[axis]:
                        in_position = False
                    else:
                        actuator = self.actuators[axis]
                        in_position = actuator.kind(tai) == actuator.Kind.Tracking
                    if not in_position and axis in axes_in_use:
                        all_in_position = False
                    await self.write_evt(
                        evt_id=self._in_position_names[axis], inPosition=in_position
                    )
                await self.write_evt(
                    evt_id=Event.ALLAXESINPOSITION, inPosition=all_in_position
                )

            # compute m3_state for use setting m3State.state
            # and m3RotatorDetentSwitches
            m3_state = None
            if m3_in_position:
                # We are either at a port or at an unknown position. Use the
                # _port_info_dict to map between exit port value and m3 state.
                # If port is not mapped use unknown position. This is needed
                # in case the exit port value does not match the m3 state
                # value.
                # TODO: DM-36825 Remove mapping once the enumeration values
                # match.
                m3_state = self._port_info_dict.get(
                    exit_port, (None, M3State.UNKNOWNPOSITION, None)
                )[1]
            elif m3actuator.kind(tai) == m3actuator.Kind.Slewing:
                m3_state = M3State.INMOTION
            else:
                m3_state = M3State.UNKNOWNPOSITION
            assert m3_state is not None

            # handle m3State
            await self.write_evt(evt_id=Event.M3STATE, state=m3_state)

            # Handle M3 detent switch
            detent_map = {
                1: "nasmyth1Active",
                2: "nasmyth2Active",
                3: "port3Active",
            }
            at_field = detent_map.get(m3_state, None)
            detent_values = dict(
                (field_name, field_name == at_field)
                for field_name in detent_map.values()
            )
            await self.write_evt(evt_id=Event.M3ROTATORDETENTSWITCHES, **detent_values)
        except Exception as e:
            print(f"update_events failed: {e}")
            raise

    async def update_telemetry(self) -> None:
        """Output all telemetry data messages."""
        try:
            # TODO DM-38912 Send "nitems" via configuration.
            nitems = 100
            curr_time = utils.current_tai()

            times = np.linspace(
                start=curr_time - self._telemetry_interval,
                stop=curr_time,
                num=nitems,
                endpoint=True,
            )

            trajectory_data = self.csc.tel_trajectory.DataType()
            azel_encoders_data = self.csc.tel_mount_AzEl_Encoders.DataType()
            nasmyth_encoders_data = self.csc.tel_mount_Nasmyth_Encoders.DataType()
            torqueDemand_data = self.csc.tel_torqueDemand.DataType()
            measuredTorque_data = self.csc.tel_measuredTorque.DataType()
            measuredMotorVelocity_data = self.csc.tel_measuredMotorVelocity.DataType()
            azel_mountMotorEncoders_data = (
                self.csc.tel_azEl_mountMotorEncoders.DataType()
            )
            nasmyth_m3_mountMotorEncoders_data = (
                self.csc.tel_nasymth_m3_mountMotorEncoders.DataType()
            )

            for i, tai in enumerate(times):
                segments = [actuator.path.at(tai) for actuator in self.actuators]
                current_position = np.array(
                    [segment.position for segment in segments], dtype=float
                )
                curr_vel = np.array(
                    [segment.velocity for segment in segments], dtype=float
                )
                curr_accel = np.array(
                    [segment.acceleration for segment in segments], dtype=float
                )

                axis_encoder_counts = (
                    (current_position * self.axis_encoder_counts_per_deg)
                    .astype(int)
                    .tolist()
                )
                torque = curr_accel * self.torque_per_accel
                motor_pos = current_position * self.motor_axis_ratio
                motor_pos = (motor_pos + 360) % 360 - 360
                motor_encoder_counts = (
                    (motor_pos * self.motor_encoder_counts_per_deg).astype(int).tolist()
                )

                trajectory_data.elevation[i] = current_position[Axis.Elevation]
                trajectory_data.azimuth[i] = current_position[Axis.Azimuth]
                trajectory_data.nasmyth1RotatorAngle[i] = current_position[Axis.NA1]
                trajectory_data.nasmyth2RotatorAngle[i] = current_position[Axis.NA2]
                trajectory_data.elevationVelocity[i] = curr_vel[Axis.Elevation]
                trajectory_data.azimuthVelocity[i] = curr_vel[Axis.Azimuth]
                trajectory_data.nasmyth1RotatorAngleVelocity[i] = curr_vel[Axis.NA1]
                trajectory_data.nasmyth2RotatorAngleVelocity[i] = curr_vel[Axis.NA2]

                azel_encoders_data.elevationCalculatedAngle[i] = current_position[
                    Axis.Elevation
                ]
                azel_encoders_data.elevationEncoder1Raw[i] = axis_encoder_counts[
                    Axis.Elevation
                ]

                azel_encoders_data.elevationEncoder2Raw[i] = axis_encoder_counts[
                    Axis.Elevation
                ]
                azel_encoders_data.elevationEncoder3Raw[i] = axis_encoder_counts[
                    Axis.Elevation
                ]
                azel_encoders_data.azimuthCalculatedAngle[i] = current_position[
                    Axis.Azimuth
                ]
                azel_encoders_data.azimuthEncoder1Raw[i] = axis_encoder_counts[
                    Axis.Azimuth
                ]
                azel_encoders_data.azimuthEncoder2Raw[i] = axis_encoder_counts[
                    Axis.Azimuth
                ]
                azel_encoders_data.azimuthEncoder3Raw[i] = axis_encoder_counts[
                    Axis.Azimuth
                ]

                nasmyth_encoders_data.nasmyth1CalculatedAngle[i] = current_position[
                    Axis.NA1
                ]
                nasmyth_encoders_data.nasmyth1Encoder1Raw[i] = axis_encoder_counts[
                    Axis.NA1
                ]
                nasmyth_encoders_data.nasmyth1Encoder2Raw[i] = axis_encoder_counts[
                    Axis.NA1
                ]
                nasmyth_encoders_data.nasmyth1Encoder3Raw[i] = axis_encoder_counts[
                    Axis.NA1
                ]
                nasmyth_encoders_data.nasmyth2CalculatedAngle[i] = current_position[
                    Axis.NA2
                ]
                nasmyth_encoders_data.nasmyth2Encoder1Raw[i] = axis_encoder_counts[
                    Axis.NA2
                ]
                nasmyth_encoders_data.nasmyth2Encoder2Raw[i] = axis_encoder_counts[
                    Axis.NA2
                ]
                nasmyth_encoders_data.nasmyth2Encoder3Raw[i] = axis_encoder_counts[
                    Axis.NA2
                ]

                torqueDemand_data.elevationMotorTorque[i] = torque[Axis.Elevation]
                torqueDemand_data.azimuthMotor1Torque[i] = torque[Axis.Azimuth]
                torqueDemand_data.azimuthMotor2Torque[i] = torque[Axis.Azimuth]
                torqueDemand_data.nasmyth1MotorTorque[i] = torque[Axis.NA1]
                torqueDemand_data.nasmyth2MotorTorque[i] = torque[Axis.NA2]

                measuredTorque_data.elevationMotorTorque[i] = torque[Axis.Elevation]
                measuredTorque_data.azimuthMotor1Torque[i] = torque[Axis.Azimuth]
                measuredTorque_data.azimuthMotor2Torque[i] = torque[Axis.Azimuth]
                measuredTorque_data.nasmyth1MotorTorque[i] = torque[Axis.NA1]
                measuredTorque_data.nasmyth2MotorTorque[i] = torque[Axis.NA2]

                measuredMotorVelocity_data.elevationMotorVelocity[i] = curr_vel[
                    Axis.Elevation
                ]
                measuredMotorVelocity_data.azimuthMotor1Velocity[i] = curr_vel[
                    Axis.Azimuth
                ]
                measuredMotorVelocity_data.azimuthMotor2Velocity[i] = curr_vel[
                    Axis.Azimuth
                ]
                measuredMotorVelocity_data.nasmyth1MotorVelocity[i] = curr_vel[Axis.NA1]
                measuredMotorVelocity_data.nasmyth2MotorVelocity[i] = curr_vel[Axis.NA2]

                azel_mountMotorEncoders_data.elevationEncoder[i] = motor_pos[
                    Axis.Elevation
                ]
                azel_mountMotorEncoders_data.azimuth1Encoder[i] = motor_pos[
                    Axis.Azimuth
                ]
                azel_mountMotorEncoders_data.azimuth2Encoder[i] = motor_pos[
                    Axis.Azimuth
                ]
                azel_mountMotorEncoders_data.elevationEncoderRaw[
                    i
                ] = motor_encoder_counts[Axis.Elevation]
                azel_mountMotorEncoders_data.azimuth1EncoderRaw[
                    i
                ] = motor_encoder_counts[Axis.Azimuth]
                azel_mountMotorEncoders_data.azimuth2EncoderRaw[
                    i
                ] = motor_encoder_counts[Axis.Azimuth]

                nasmyth_m3_mountMotorEncoders_data.nasmyth1Encoder[i] = motor_pos[
                    Axis.NA1
                ]
                nasmyth_m3_mountMotorEncoders_data.nasmyth2Encoder[i] = motor_pos[
                    Axis.NA2
                ]
                nasmyth_m3_mountMotorEncoders_data.m3Encoder[i] = motor_pos[Axis.M3]
                nasmyth_m3_mountMotorEncoders_data.nasmyth1EncoderRaw[
                    i
                ] = motor_encoder_counts[Axis.NA1]
                nasmyth_m3_mountMotorEncoders_data.nasmyth2EncoderRaw[
                    i
                ] = motor_encoder_counts[Axis.NA2]
                nasmyth_m3_mountMotorEncoders_data.m3EncoderRaw[
                    i
                ] = motor_encoder_counts[Axis.M3]

            await self.write_telemetry(
                tel_id=Telemetry.TRAJECTORY,
                timestamp=times[0],
                data=trajectory_data,
            )
            await self.write_telemetry(
                tel_id=Telemetry.MOUNT_AZEL_ENCODERS,
                timestamp=times[0],
                data=azel_encoders_data,
            )
            await self.write_telemetry(
                tel_id=Telemetry.MOUNT_NASMYTH_ENCODERS,
                timestamp=times[0],
                data=nasmyth_encoders_data,
            )
            await self.write_telemetry(
                tel_id=Telemetry.TORQUE_DEMAND,
                timestamp=times[0],
                data=torqueDemand_data,
            )
            await self.write_telemetry(
                tel_id=Telemetry.MEASURED_TORQUE,
                timestamp=times[0],
                data=measuredTorque_data,
            )
            await self.write_telemetry(
                tel_id=Telemetry.MEASURED_MOTOR_VELOCITY,
                timestamp=times[0],
                data=measuredMotorVelocity_data,
            )
            await self.write_telemetry(
                tel_id=Telemetry.AZEL_MOTOR_MOUNT_ENCODERS,
                timestamp=times[0],
                data=azel_mountMotorEncoders_data,
            )
            await self.write_telemetry(
                tel_id=Telemetry.NASMYTH_M3_MOUNT_MOTOR_ENCODERS,
                timestamp=times[0],
                data=nasmyth_m3_mountMotorEncoders_data,
            )
        except Exception as e:
            print(f"update_telemetry failed: {e}")
            raise

    async def write_response(self, response: str, sequence_id: int) -> None:
        data = {CommandKey.ID: response, CommandKey.SEQUENCE_ID: sequence_id}
        await self.cmd_evt_server.write_json(data=data)

    async def write_ack_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.ACK, sequence_id)

    async def write_fail_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.FAIL, sequence_id)

    async def write_noack_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.NOACK, sequence_id)

    async def write_success_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.SUCCESS, sequence_id)

    def get_items_from_data(
        self, data: salobj.topics.BaseTopic
    ) -> dict[str, typing.Any]:
        # This needs to be cleaned up as soon as we have moved to Kafka.
        data_dict = data.get_vars() if hasattr(data, "get_vars") else vars(data)
        items = {k: v for k, v in data_dict.items() if not k.startswith("private")}
        return items

    async def write_evt(self, evt_id: str, **kwargs: typing.Any) -> None:
        await self.cmd_evt_server.write_json(data={"id": evt_id, **kwargs})

    async def write_telemetry(
        self, tel_id: str, timestamp: np.float64, data: salobj.topics.BaseTopic
    ) -> None:
        data.cRIO_timestamp = timestamp.item()
        items = self.get_items_from_data(data)
        items["id"] = tel_id
        await self.telemetry_server.write_json(data={**items})

    async def disable_all_drives(self) -> None:
        """Stop all drives, disable them and put on brakes."""
        self._tracking_enabled = False
        already_stopped = True
        tai = utils.current_tai()
        for axis in Axis:
            actuator = self.actuators[axis]
            if actuator.kind(tai) == actuator.Kind.Stopped:
                self._axis_enabled[axis] = False
            else:
                already_stopped = False
                actuator.stop()
        self._disable_all_drives_task.cancel()
        if not already_stopped:
            self._disable_all_drives_task = asyncio.create_task(
                self._finish_disable_all_drives()
            )
        await self.update_events()

    async def _finish_disable_all_drives(self) -> None:
        """Wait for the main axes to stop."""
        end_times = [actuator.path[-1].tai for actuator in self.actuators]
        max_end_time = max(end_times)
        # give a bit of margin to be sure the axes are stopped
        dt = 0.1 + max_end_time - utils.current_tai()
        if dt > 0:
            await asyncio.sleep(dt)
        for axis in Axis:
            self._axis_enabled[axis] = False

    async def _finish_stop_tracking(self) -> None:
        """Wait for the main axes to stop."""
        end_times = [self.actuators[axis].path[-1].tai for axis in MainAxes]
        max_end_time = max(end_times)
        dt = 0.1 + max_end_time - utils.current_tai()
        if dt > 0:
            await asyncio.sleep(dt)

    async def start_tasks(self) -> None:
        if self._events_and_telemetry_task.done():
            self._events_and_telemetry_task = asyncio.create_task(
                self.events_and_telemetry_loop()
            )

    async def stop_tasks(self) -> None:
        self._disable_all_drives_task.cancel()
        self._stop_tracking_task.cancel()
        self._events_and_telemetry_task.cancel()
        self._kill_tracking_timer.cancel()

    async def events_and_telemetry_loop(self) -> None:
        """Output telemetry and events that have changed

        Notes
        -----
        See `update_telemetry` for the telemetry that is output.

        See `update_events` for the events that are output.
        """
        i = 0
        while True:
            # Update events first so that limits are handled.
            i += 1
            await self.update_events()

            if i >= self._events_per_telemetry:
                i = 0
                await self.update_telemetry()

            await asyncio.sleep(self._telemetry_interval / self._events_per_telemetry)

    def __enter__(self) -> None:
        # This class only implements an async context manager.
        raise NotImplementedError("Use 'async with' instead.")

    def __exit__(
        self,
        type: typing.Type[BaseException],
        value: BaseException,
        traceback: types.TracebackType,
    ) -> None:
        # __exit__ should exist in pair with __enter__ but never be executed.
        raise NotImplementedError("Use 'async with' instead.")

    async def __aenter__(self) -> McsSimulator:
        return self

    async def __aexit__(
        self,
        type: typing.Type[BaseException],
        value: BaseException,
        traceback: types.TracebackType,
    ) -> None:
        await self.cmd_evt_server.close()
        await self.telemetry_server.close()
