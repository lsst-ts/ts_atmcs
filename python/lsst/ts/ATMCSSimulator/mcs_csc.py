# This file is part of ts_ATMCSSimulator.
#
# Developed for the LSST Data Management System.
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

__all__ = ["ATMCSCsc", "Axis", "MainAxes"]

import asyncio
import enum
import time

import numpy as np

from lsst.ts import salobj
from . import path
from .actuator import Actuator

import SALPY_ATMCS


class Axis(enum.IntEnum):
    Elevation = 0
    Azimuth = 1
    NA1 = 2
    NA2 = 3
    M3 = 4


MainAxes = (Axis.Elevation, Axis.Azimuth, Axis.NA1, Axis.NA2)


class ATMCSCsc(salobj.BaseCsc):
    """Simulator for auxiliary telescope motor control system CSC.

    Parameters
    ----------
    initial_state : `salobj.State` or `int` (optional)
        The initial state of the CSC. This is provided for unit testing,
        as real CSCs should start up in `State.STANDBY`, the default.
    initial_simulation_mode : `int` (optional)
        Initial simulation mode.
        The only allowed value is 1: simulating.

    Notes
    -----
    .. _axis:

    The axes are, in order:
    - elevation
    - azimuth
    - Nasmyth1 rotator
    - Nasmyth2 rotator
    - m3 rotator

    **Limitations**

    * Jerk is infinite.
    * When an axis has multiple motors or encoders, all are treated as
      identical (e.g. report identical positions).
    * The only way to hit a limit switch is to configure the position
      of that switch within the command limits for that axis.
    * The CSC always wakes up at position 0 for all axes except elevation,
      and at the minimum allowed position for elevation.
    * The model for the azimuth topple block is primitive:

      * The CCW switch is active if az < az1
      * The CW switch is  active if az > az2
      * Otherwise neither switch is active
    """
    def __init__(self, initial_state=salobj.State.STANDBY, initial_simulation_mode=1):
        super().__init__(SALPY_ATMCS, index=0, initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)
        self.telemetry_interval = 0.1
        """Interval between telemetry updates (sec)"""
        self._telemetry_task = None
        """Task that runs while the telemetry loop sleeps."""
        self._stop_tracking_task = None
        """Task that runs while axes are slewing due to stopTracking."""
        self._disable_all_drives_task = None
        """Task that runs while axes are halting before being disabled."""
        self._port_info_dict = {
            SALPY_ATMCS.ATMCS_shared_M3ExitPort_Nasmyth1: (0, SALPY_ATMCS.ATMCS_shared_M3State_Nasmyth1),
            SALPY_ATMCS.ATMCS_shared_M3ExitPort_Nasmyth2: (1, SALPY_ATMCS.ATMCS_shared_M3State_Nasmyth2),
            SALPY_ATMCS.ATMCS_shared_M3ExitPort_Port3: (2, SALPY_ATMCS.ATMCS_shared_M3State_Port3),
        }
        """Dict of port enum: (port_az index, M3State enum constant)"""

        self._min_lim_names = (
            "elevationLimitSwitchLower",
            "azimuthLimitSwitchCW",
            "nasmyth1LimitSwitchCW",
            "nasmyth2LimitSwitchCW",
            "m3RotatorLimitSwitchCW",
        )
        """Name of minimum limit switch event for each axis."""

        self._max_lim_names = (
            "elevationLimitSwitchUpper",
            "azimuthLimitSwitchCCW",
            "nasmyth1LimitSwitchCCW",
            "nasmyth2LimitSwitchCCW",
            "m3RotatorLimitSwitchCCW",
        )
        """Name of maximum limit switch event for each axis."""

        self._in_position_names = (
            "elevationInPosition",
            "azimuthInPosition",
            "nasmyth1RotatorInPosition",
            "nasmyth2RotatorInPosition",
            "m3InPosition",
        )
        """Name of "in position" event for each axis;
        excludes ``allAxesInPosition``.
        """

        self._drive_status_names = (
            ("elevationDriveStatus",),
            ("azimuthDrive1Status", "azimuthDrive2Status",),
            ("nasmyth1DriveStatus",),
            ("nasmyth2DriveStatus",),
            ("m3DriveStatus",),
        )
        """Name of drive status events for each axis."""

        self._brake_names = (
            ("elevationBrake",),
            ("azimuthBrake1", "azimuthBrake2",),
            ("nasmyth1Brake",),
            ("nasmyth2Brake",),
            (),
        )
        """Name of brake events for each axis."""

        self._tracking_enabled = False
        """Has tracking been enabled by startTracking?

        This remains true until stopTracking is called or the
        summary state is no longer salobj.State.Enabled,
        even if some drives have been disabled by running into limits.
        """

        self._axis_enabled = np.zeros([5], dtype=bool)
        """Is this particular axis enabled?

        This remains true until stopTracking is called or the
        summary state is no longer salobj.State.Enabled,
        or the axis runs into a limit.
        """

        self._axis_braked = np.zeros([5], dtype=bool)
        """If True the axis has its brakes (if any) applied

        In this state _axis_enabled should be False
        and the actuator should be halted.
        """

        self._kill_tracking_timer = None
        """Timer to kill tracking if trackTarget doesn't arrive in time.
        """

        self.configure()
        # note: initial events are output by report_summary_state

    def configure(self,
                  max_tracking_interval=2.5,
                  pmin_cmd=(5, -270, -165, -165, 0),
                  pmax_cmd=(90, 270, 165, 165, 180),
                  pmin_lim=(3, -272, -167, -167, -2),
                  pmax_lim=(92, 272, 167, 167, 182),
                  vmax=(5, 5, 5, 5, 5),
                  amax=(3, 3, 3, 3, 3),
                  topple_az=(2, 5),
                  port_az=(0, 180, 90),
                  needed_in_pos=3,
                  axis_encoder_counts_per_deg=(3.6e6, 3.6e6, 3.6e6, 3.6e6, 3.6e6),
                  motor_encoder_counts_per_deg=(3.6e5, 3.6e5, 3.6e5, 3.6e5, 3.6e5),
                  motor_axis_ratio=(100, 100, 100, 100, 100),
                  torque_per_accel=(1, 1, 1, 1, 1),
                  nsettle=2,
                  limit_overtravel=1,
                  ):
        """Set configuration.

        Parameters
        ----------
        max_tracking_interval : `float`
            Maximum time between tracking updates (sec)
        pmin_cmd : ``iterable`` of 5 `float`
            Minimum commanded position for each axis_, in deg
        pmax_cmd : ``iterable`` of 5 `float`
            Minimum commanded position for each axis_, in deg
        pmin_lim : ``iterable`` of 5 `float`
            Position of minimum limit switch for each axis_, in deg
        pmax_lim : ``iterable`` of 5 `float`
            Position of maximum limit switch for each axis_, in deg
        vmax : ``iterable`` of 5 `float`
            Maximum velocity of each axis, in deg/sec
        amax : ``iterable`` of 5 `float`
            Maximum acceleration of each axis, in deg/sec
        topple_az : ``iterable`` of 2 `float`
            Min, max azimuth at which the topple block moves, in deg
        port_az : ``iterable`` of 3 `float`
            Position of each instrument port: NA1, NA2 and Port3
        axis_encoder_counts_per_deg : `list` [`float`]
            Axis encoder resolution, for each axis_, in counts/deg
        motor_encoder_counts_per_deg : `list` [`float`]
            Motor encoder resolution, for each axis_, in counts/deg
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
        """
        def convert_values(name, values, nval):
            out = np.array(values, dtype=float)
            if out.shape != (nval,):
                raise salobj.ExpectedError(f"Could not format {name}={values!r} as {nval} floats")
            return out

        # convert and check all values first,
        # so nothing changes if any input is invalid
        pmin_cmd = convert_values("pmin_cmd", pmin_cmd, 5)
        pmax_cmd = convert_values("pmax_cmd", pmax_cmd, 5)
        pmin_lim = convert_values("pmin_lim", pmin_lim, 5)
        pmax_lim = convert_values("pmax_lim", pmax_lim, 5)
        vmax = convert_values("vmax", vmax, 5)
        amax = convert_values("amax", amax, 5)
        if vmax.min() <= 0:
            raise salobj.ExpectedError(f"vmax={vmax}; all values must be positive")
        if amax.min() <= 0:
            raise salobj.ExpectedError(f"amax={amax}; all values must be positive")
        topple_az = convert_values("topple_az", topple_az, 2)
        port_az = convert_values("port_az", port_az, 3)
        axis_encoder_counts_per_deg = convert_values("axis_encoder_counts_per_deg",
                                                     axis_encoder_counts_per_deg, 5)
        motor_encoder_counts_per_deg = convert_values("motor_encoder_counts_per_deg",
                                                      motor_encoder_counts_per_deg, 5)
        motor_axis_ratio = convert_values("motor_axis_ratio", motor_axis_ratio, 5)
        torque_per_accel = convert_values("torque_per_accel", torque_per_accel, 5)
        if limit_overtravel < 0:
            raise ValueError(f"limit_overtravel={limit_overtravel} must be >= 0")

        self.max_tracking_interval = max_tracking_interval
        self.pmin_cmd = pmin_cmd
        self.pmax_cmd = pmax_cmd
        self.pmin_lim = pmin_lim
        self.pmax_lim = pmax_lim
        self.vmax = vmax
        self.topple_az = topple_az
        self.port_az = port_az
        self.axis_encoder_counts_per_deg = axis_encoder_counts_per_deg
        self.motor_encoder_counts_per_deg = motor_encoder_counts_per_deg
        self.motor_axis_ratio = motor_axis_ratio
        self.torque_per_accel = torque_per_accel
        self.nsettle = nsettle
        self.m3tolerance = 1e-5
        self.limit_overtravel = limit_overtravel
        """Allowed position error for M3 to be considered in position (deg)"""

        t = time.time()
        self.actuators = [Actuator(
            pmin=self.pmin_cmd[axis], pmax=self.pmax_cmd[axis],
            vmax=vmax[axis], amax=amax[axis],
            dtmax_track=self.max_tracking_interval,
            nsettle=self.nsettle, t=t,
        ) for axis in Axis]

    def do_startTracking(self, id_data):
        self.assert_enabled("startTracking")
        if not self.evt_m3InPosition.data.inPosition:
            raise salobj.ExpectedError("Cannot startTracking until M3 is at a known position")
        if self._stop_tracking_task and not self._stop_tracking_task.done():
            raise salobj.ExpectedError("stopTracking not finished yet")
        self._tracking_enabled = True
        self.update_events()
        self._set_tracking_timer(restart=True)

    def do_trackTarget(self, id_data):
        self.assert_enabled("trackTarget")
        if not self._tracking_enabled:
            raise salobj.ExpectedError("Cannot trackTarget until tracking is enabled")
        data = id_data.data
        try:
            if data.azimuthDirection == SALPY_ATMCS.ATMCS_shared_AzimuthDirection_ClockWise:
                wrap_pos = True
            elif data.azimuthDirection == SALPY_ATMCS.ATMCS_shared_AzimuthDirection_CounterClockWise:
                wrap_pos = False
            else:
                raise salobj.ExpectedError(f"azimuthDirection={data.azimuthDirection}; must be 1 or 2")
            pos = np.array([data.elevation, data.azimuth,
                            data.nasmyth1RotatorAngle, data.nasmyth2RotatorAngle], dtype=float)
            if not 0 <= pos[1] <= 360:
                raise salobj.ExpectedError(f"azimuth={data.azimuth}; must be in range 0 - 360")
            pos[1] = path.wrap_angle(pos[1], wrap_pos, self.pmin_cmd[1], self.pmax_cmd[1])
            vel = np.array([data.elevationVelocity, data.azimuthVelocity,
                           data.nasmyth1RotatorAngleVelocity, data.nasmyth2RotatorAngleVelocity], dtype=float)
            dt = time.time() - data.time
            curr_pos = pos + dt*vel
            if np.any(curr_pos < self.pmin_cmd[0:4]) or np.any(curr_pos > self.pmax_cmd[0:4]):
                raise salobj.ExpectedError(f"One or more target positions {curr_pos} not in range "
                                           f"{self.pmin_cmd} to {self.pmax_cmd} at the current time")
            if np.any(np.abs(vel) > self.vmax[0:4]):
                raise salobj.ExpectedError(f"Magnitude of one or more target velocities "
                                           f"{vel} > {self.pmax_vel}")
        except Exception as e:
            self.evt_errorCode.set_put(errorCode=1, errorReport=f"trackTarget failed: {e}", force_output=True)
            self.fault()
            raise

        for i in range(4):
            self.actuators[i].set_cmd(pos=pos[i], vel=vel[i], t=data.time)
        self._set_tracking_timer(restart=True)

    def _set_tracking_timer(self, restart):
        """Restart or stop the tracking timer.

        Parameters
        ----------
        restart : `bool`
            If True then start or restart the tracking timer, else stop it.
        """
        if self._kill_tracking_timer and not self._kill_tracking_timer.done():
            self._kill_tracking_timer.cancel()
        if restart:
            self._kill_tracking_timer = asyncio.ensure_future(self.kill_tracking())

    def do_setInstrumentPort(self, id_data):
        self.assert_enabled("setInstrumentPort")
        if self._tracking_enabled:
            raise salobj.ExpectedError("Cannot setInstrumentPort while tracking is enabled")
        port = id_data.data.port
        try:
            port_az_ind = self._port_info_dict[port][0]
        except IndexError:
            raise salobj.ExpectedError(f"Invalid port={port}")
        try:
            port_az = self.port_az[port_az_ind]
        except IndexError:
            raise RuntimeError(f"Bug! invalid port_az_ind={port_az_ind} for port={port}")
        self.actuators[Axis.M3].set_cmd(pos=port_az, vel=0, t=time.time())
        self.evt_m3PortSelected.set_put(selected=port)

    async def do_stopTracking(self, id_data):
        self.assert_enabled("stopTracking")
        if self._stop_tracking_task and not self._stop_tracking_task.done():
            raise salobj.ExpectedError("Already stopping")
        self._set_tracking_timer(restart=False)
        self._tracking_enabled = False
        for axis in MainAxes:
            self.actuators[axis].stop()
        if self._stop_tracking_task is not None and not self._stop_tracking_task.done():
            self._stop_tracking_task.cancel()
        self._stop_tracking_task = asyncio.ensure_future(self._finish_stop_tracking())
        self.update_events()

    async def kill_tracking(self):
        """Wait ``self.max_tracking_interval`` seconds and disable tracking.

        Intended for use by `do_trackTarget` to abort tracking
        if the next ``trackTarget`` command is not seen quickly enough.
        """
        await asyncio.sleep(self.max_tracking_interval)
        self.evt_errorCode.set_put(errorCode=2,
                                   errorReport=f"trackTarget not seen in {self.max_tracking_interval} sec",
                                   force_output=True)
        self.fault()

    def disable_all_drives(self):
        """Stop all drives, disable them and put on brakes.
        """
        self._tracking_enabled = False
        already_stopped = True
        t0 = time.time()
        for axis in Axis:
            actuator = self.actuators[axis]
            if actuator.kind(t0) == path.Kind.Stopped:
                self._axis_enabled[axis] = False
                self._axis_braked[axis] = True
            else:
                already_stopped = False
                actuator.stop()
        if self._disable_all_drives_task is not None and not self._disable_all_drives_task:
            self._disable_all_drives_task.cancel()
        if not already_stopped:
            self._disable_all_drives_task = asyncio.ensure_future(self._finish_disable_all_drives())
        self.update_events()

    async def _finish_disable_all_drives(self):
        """Wait for the main axes to stop.
        """
        end_times = [actuator.curr[-1].t0 for actuator in self.actuators]
        max_end_time = max(end_times)
        # give a bit of margin to be sure the axes are stopped
        dt = 0.1 + max_end_time - time.time()
        if dt > 0:
            await asyncio.sleep(dt)
        for axis in Axis:
            self._axis_enabled[axis] = False
            self._axis_braked[axis] = True
        self.update_events()

    async def _finish_stop_tracking(self):
        """Wait for the main axes to stop.
        """
        try:
            end_times = [self.actuators[axis].curr[-1].t0 for axis in MainAxes]
            max_end_time = max(end_times)
            dt = 0.1 + max_end_time - time.time()
            if dt > 0:
                await asyncio.sleep(dt)
        finally:
            self.update_events()

    def update_events(self):
        """Set state of the various events.

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
        t = time.time()
        curr_pos = np.array([actuator.curr.pva(t)[0] for actuator in self.actuators], dtype=float)
        m3actuator = self.actuators[Axis.M3]

        # Handle M3 actuator; set_cmd needs to be called to transition
        # from slewing to tracking, and that is done here for M3
        # (the trackPosition command does that for the other axes).
        if m3actuator.kind(t) == path.Kind.Slewing and t > m3actuator.curr[-1].t0:
            m3actuator.set_cmd(pos=m3actuator.curr[-1].p0, vel=0, t=t)

        # Handle limit switches
        # including aborting axes that are out of limits
        # and putting on their brakes (if any)
        abort_axes = []
        for axis in Axis:
            self.set_event(self._min_lim_names[axis], active=curr_pos[axis] < self.pmin_lim[axis])
            self.set_event(self._max_lim_names[axis], active=curr_pos[axis] > self.pmax_lim[axis])
            if curr_pos[axis] < self.pmin_lim[axis] or curr_pos[axis] > self.pmax_lim[axis]:
                abort_axes.append(axis)
        for axis in abort_axes:
            pos = curr_pos[axis]
            pos = max(pos, self.pmin_lim[axis] - self.limit_overtravel)
            pos = min(pos, self.pmax_lim[axis] + self.limit_overtravel)
            self.actuators[axis].abort(t=t, pos=pos)
            self._axis_enabled[axis] = False
            self._axis_braked[axis] = True

        # Handle brakes
        for axis in Axis:
            for brake_name in self._brake_names[axis]:
                self.set_event(brake_name, engaged=self._axis_braked[axis])

        # Handle drive status (which means enabled)
        for axis in Axis:
            for evt_name in self._drive_status_names[axis]:
                self.set_event(evt_name, enable=self._axis_enabled[axis])

        # Handle atMountState
        if self._tracking_enabled:
            mount_state = SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled
        elif self._stop_tracking_task and not self._stop_tracking_task.done():
            mount_state = SALPY_ATMCS.ATMCS_shared_AtMountState_Stopping
        else:
            mount_state = SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled
        self.evt_atMountState.set_put(state=mount_state)

        # Handle azimuth topple block
        if curr_pos[Axis.Azimuth] < self.topple_az[0]:
            self.evt_azimuthToppleBlockCCW.set_put(active=True)
            self.evt_azimuthToppleBlockCW.set_put(active=False)
        elif curr_pos[Axis.Azimuth] > self.topple_az[1]:
            self.evt_azimuthToppleBlockCCW.set_put(active=False)
            self.evt_azimuthToppleBlockCW.set_put(active=True)
        else:
            self.evt_azimuthToppleBlockCCW.set_put(active=False)
            self.evt_azimuthToppleBlockCW.set_put(active=False)

        # Handle m3InPosition
        # M3 is in position if the current velocity is 0
        # and the current position equals the commanded position.
        m3cmd_pos = m3actuator.cmd.p0
        m3curr_pos, m3curr_vel = m3actuator.curr[-1].pva(t)[0:2]
        m3_in_position = m3curr_vel == 0 and abs(m3cmd_pos - m3curr_pos) < self.m3tolerance
        self.evt_m3InPosition.set_put(inPosition=m3_in_position)

        # Handle "in position" events for the main axes.
        # Main axes are in position if enabled, the brakes are off,
        # and actuator.kind(t) is tracking.
        if not self._tracking_enabled:
            for axis in MainAxes:
                self.set_event(self._in_position_names[axis], inPosition=False)
            self.evt_allAxesInPosition.set_put(inPosition=False)
        else:
            all_in_position = m3_in_position
            for axis in MainAxes:
                if self._axis_braked[axis] or not self._axis_enabled[axis]:
                    in_position = False
                else:
                    kind = self.actuators[axis].kind(t)
                    in_position = kind == path.Kind.Tracking
                all_in_position &= in_position
                self.set_event(self._in_position_names[axis], inPosition=in_position)
            self.evt_allAxesInPosition.set_put(inPosition=all_in_position)

        # compute m3_state for use setting m3State.state
        # and m3RotatorDetentSwitches
        m3_state = None
        if m3_in_position:
            # we are either at a port or at an unknown position
            for port_enum, (port_ind, state_enum) in self._port_info_dict.items():
                port_az = self.port_az[port_ind]
                if abs(m3curr_pos - port_az) < self.m3tolerance:
                    m3_state = state_enum
                    break
            else:
                # Move is finished, but not at a known point
                m3_state = SALPY_ATMCS.ATMCS_shared_M3State_UnknownPosition
        elif m3actuator.kind(t) == path.Kind.Slewing:
            m3_state = SALPY_ATMCS.ATMCS_shared_M3State_InMotion
        else:
            m3_state = SALPY_ATMCS.ATMCS_shared_M3State_UnknownPosition
        assert m3_state is not None

        # handle m3State
        self.evt_m3State.set_put(state=m3_state)

        # Handle M3 detent switch
        detent_map = {
            SALPY_ATMCS.ATMCS_shared_M3State_Nasmyth1: "nasmyth1Active",
            SALPY_ATMCS.ATMCS_shared_M3State_Nasmyth2: "nasmyth2Active",
            SALPY_ATMCS.ATMCS_shared_M3State_Port3: "port3Active",
        }
        at_field = detent_map.get(m3_state, None)
        detent_values = dict((field_name, field_name == at_field) for field_name in detent_map.values())
        self.evt_m3RotatorDetentSwitches.set_put(**detent_values)

    async def implement_simulation_mode(self, simulation_mode):
        if simulation_mode != 1:
            raise salobj.ExpectedError(
                f"This CSC only supports simulation; simulation_mode={simulation_mode} but must be 1")

    def set_event(self, evt_name, **kwargs):
        """Call ``ControllerEvent.set_put`` for an event specified by name.

        Parameters
        ----------
        evt_name : `str`
            Event name (without the ``evt_`` prefix)
        **kwargs : `dict`
            Data for ``ControllerEvent.set``
        """
        evt = getattr(self, f"evt_{evt_name}")
        evt.set_put(**kwargs)

    def report_summary_state(self):
        super().report_summary_state()
        if self.summary_state == salobj.State.ENABLED:
            for axis in Axis:
                self._axis_enabled[axis] = True
                self._axis_braked[axis] = False
        else:
            self.disable_all_drives()
        if self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            asyncio.ensure_future(self.telemetry_loop())

    async def telemetry_loop(self):
        """Output telemetry and events that have changed

        Notes
        -----
        Here are the telemetry topics that are output:

        * mountEncoders
        * torqueDemand
        * measuredTorque
        * measuredMotorVelocity
        * mountMotorEncoders

        See `update_events` for the events that are output.
        """
        if self._telemetry_task is not None and not self._telemetry_task.done():
            self._telemetry_task.cancel()
        while self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            # update events first so that limits are handled
            self.update_events()

            curr_time = time.time()
            pva_list = [actuator.curr.pva(curr_time) for actuator in self.actuators]
            curr_pos = np.array([pva[0] for pva in pva_list], dtype=float)
            curr_vel = np.array([pva[1] for pva in pva_list], dtype=float)
            curr_accel = np.array([pva[2] for pva in pva_list], dtype=float)

            axis_encoder_counts = (curr_pos*self.axis_encoder_counts_per_deg).astype(int)
            torque = curr_accel*self.torque_per_accel
            motor_pos = curr_pos * self.motor_axis_ratio
            motor_pos = (motor_pos + 360) % 360 - 360
            motor_encoder_counts = (motor_pos*self.motor_encoder_counts_per_deg).astype(int)

            self.tel_mountEncoders.set_put(
                elevationCalculatedAngle=curr_pos[Axis.Elevation],
                azimuthCalculatedAngle=curr_pos[Axis.Azimuth],
                nasmyth1CalculatedAngle=curr_pos[Axis.NA1],
                nasmyth2CalculatedAngle=curr_pos[Axis.NA2],
                elevationEncoder1Raw=axis_encoder_counts[Axis.Elevation],
                elevationEncoder2Raw=axis_encoder_counts[Axis.Elevation],
                elevationEncoder3Raw=axis_encoder_counts[Axis.Elevation],
                azimuthEncoder1Raw=axis_encoder_counts[Axis.Azimuth],
                azimuthEncoder2Raw=axis_encoder_counts[Axis.Azimuth],
                azimuthEncoder3Raw=axis_encoder_counts[Axis.Azimuth],
                nasmyth1Encoder1Raw=axis_encoder_counts[Axis.NA1],
                nasmyth1Encoder2Raw=axis_encoder_counts[Axis.NA1],
                nasmyth1Encoder3Raw=axis_encoder_counts[Axis.NA1],
                nasmyth2Encoder1Raw=axis_encoder_counts[Axis.NA2],
                nasmyth2Encoder2Raw=axis_encoder_counts[Axis.NA2],
                nasmyth2Encoder3Raw=axis_encoder_counts[Axis.NA2],
                trackId=self.cmd_trackTarget.data.trackId,
            )
            self.tel_torqueDemand.set_put(
                elevationMotorTorque=torque[Axis.Elevation],
                azimuthMotor1Torque=torque[Axis.Azimuth],
                azimuthMotor2Torque=torque[Axis.Azimuth],
                nasmyth1MotorTorque=torque[Axis.NA1],
                nasmyth2MotorTorque=torque[Axis.NA2],
            )
            self.tel_measuredTorque.set_put(
                elevationMotorTorque=torque[Axis.Elevation],
                azimuthMotor1Torque=torque[Axis.Azimuth],
                azimuthMotor2Torque=torque[Axis.Azimuth],
                nasmyth1MotorTorque=torque[Axis.NA1],
                nasmyth2MotorTorque=torque[Axis.NA2],
            )
            self.tel_measuredMotorVelocity.set_put(
                elevationMotorVelocity=curr_vel[Axis.Elevation],
                azimuthMotor1Velocity=curr_vel[Axis.Azimuth],
                azimuthMotor2Velocity=curr_vel[Axis.Azimuth],
                nasmyth1MotorVelocity=curr_vel[Axis.NA1],
                nasmyth2MotorVelocity=curr_vel[Axis.NA2],
            )
            self.tel_mountMotorEncoders.set_put(
                elevationEncoder=motor_pos[Axis.Elevation],
                azimuth1Encoder=motor_pos[Axis.Azimuth],
                azimuth2Encoder=motor_pos[Axis.Azimuth],
                nasmyth1Encoder=motor_pos[Axis.NA1],
                nasmyth2Encoder=motor_pos[Axis.NA2],
                elevationEncoderRaw=motor_encoder_counts[Axis.Elevation],
                azimuth1EncoderRaw=motor_encoder_counts[Axis.Azimuth],
                azimuth2EncoderRaw=motor_encoder_counts[Axis.Azimuth],
                nasmyth1EncoderRaw=motor_encoder_counts[Axis.NA1],
                nasmyth2EncoderRaw=motor_encoder_counts[Axis.NA2],
            )
            self._telemetry_task = asyncio.ensure_future(asyncio.sleep(self.telemetry_interval))
            await self._telemetry_task
