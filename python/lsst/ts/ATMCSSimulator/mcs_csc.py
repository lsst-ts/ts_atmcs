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

__all__ = ["ATMCSCsc", "Axis", "MainAxes", "curr_tai"]

import asyncio
import enum

import numpy as np

from lsst.ts import salobj
from lsst.ts.idl.enums.ATMCS import AtMountState, M3ExitPort, M3State
from . import path
from .actuator import Actuator, curr_tai


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
        super().__init__(name="ATMCS", index=0, initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)
        # interval between telemetry updates (sec)
        self._telemetry_interval = 1
        # number of event updates per telemetry update
        self._events_per_telemetry = 10
        # task that runs while the events_and_telemetry_loop runs
        self._events_and_telemetry_task = salobj.make_done_future()
        # task that runs while axes are slewing to a halt from stopTracking
        self._stop_tracking_task = salobj.make_done_future()
        # task that runs while axes are halting before being disabled
        self._disable_all_drives_task = salobj.make_done_future()
        self._port_info_dict = {
            M3ExitPort.NASMYTH1: (0, M3State.NASMYTH1, Axis.NA1),
            M3ExitPort.NASMYTH2: (1, M3State.NASMYTH2, Axis.NA2),
            M3ExitPort.PORT3: (2, M3State.PORT3, None),
        }
        """Dict of M3ExitPort (the instrument port M3 points to): tuple of:
        * index of self.m3_port_pos: the M3 position for this port
        * M3State (state of M3 axis when pointing to this port)
        * Rotator axis at this port, as an Axis enum,
          or None if this port has no rotator.
        """

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

        Note that the brakes automatically come on/off
        if the axis is disabled/enabled, respectively.
        """

        self._kill_tracking_timer = salobj.make_done_future()
        """Timer to kill tracking if trackTarget doesn't arrive in time.
        """

        self.configure()
        # note: initial events are output by report_summary_state

    async def close_tasks(self):
        await super().close_tasks()
        self._disable_all_drives_task.cancel()
        self._stop_tracking_task.cancel()
        self._events_and_telemetry_task.cancel()
        self._kill_tracking_timer.cancel()

    def configure(self,
                  max_tracking_interval=2.5,
                  pmin_cmd=(5, -270, -165, -165, 0),
                  pmax_cmd=(90, 270, 165, 165, 180),
                  pmin_lim=(3, -272, -167, -167, -2),
                  pmax_lim=(92, 272, 167, 167, 182),
                  vmax=(5, 5, 5, 5, 5),
                  amax=(3, 3, 3, 3, 3),
                  topple_az=(2, 5),
                  m3_port_pos=(0, 180, 90),
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
        m3_port_pos : ``iterable`` of 3 `float`
            M3 position of instrument ports NA1, NA2 and Port3,
            in that order.
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
        m3_port_pos = convert_values("m3_port_pos", m3_port_pos, 3)
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
        self.m3_port_pos = m3_port_pos
        self.axis_encoder_counts_per_deg = axis_encoder_counts_per_deg
        self.motor_encoder_counts_per_deg = motor_encoder_counts_per_deg
        self.motor_axis_ratio = motor_axis_ratio
        self.torque_per_accel = torque_per_accel
        self.nsettle = nsettle
        # allowed position error for M3 to be considered in position (deg)
        self.m3tolerance = 1e-5
        self.limit_overtravel = limit_overtravel

        t = curr_tai()
        self.actuators = [Actuator(
            pmin=self.pmin_cmd[axis], pmax=self.pmax_cmd[axis],
            vmax=vmax[axis], amax=amax[axis],
            # Use 0 for M3 to prevent tracking.
            # TODO DM-21957: replace M3 actuator with a point to point actuator
            dtmax_track=0 if axis == 4 else self.max_tracking_interval,
            nsettle=self.nsettle, t=t,
        ) for axis in Axis]
        self.actuators[0].verbose = True

        self.evt_positionLimits.set_put(minimum=pmin_cmd, maximum=pmax_cmd, force_output=True)

    def do_startTracking(self, data):
        self.assert_enabled("startTracking")
        if not self.evt_m3InPosition.data.inPosition:
            raise salobj.ExpectedError("Cannot startTracking until M3 is at a known position")
        if not self._stop_tracking_task.done():
            raise salobj.ExpectedError("stopTracking not finished yet")
        self._tracking_enabled = True
        self.update_events()
        self._set_tracking_timer(restart=True)

    def do_trackTarget(self, data):
        self.assert_enabled("trackTarget")
        if not self._tracking_enabled:
            raise salobj.ExpectedError("Cannot trackTarget until tracking is enabled")
        try:
            pos = np.array([data.elevation, data.azimuth,
                            data.nasmyth1RotatorAngle, data.nasmyth2RotatorAngle], dtype=float)
            vel = np.array([data.elevationVelocity, data.azimuthVelocity,
                           data.nasmyth1RotatorAngleVelocity, data.nasmyth2RotatorAngleVelocity], dtype=float)
            dt = curr_tai() - data.time
            curr_pos = pos + dt*vel
            if np.any(curr_pos < self.pmin_cmd[0:4]) or np.any(curr_pos > self.pmax_cmd[0:4]):
                raise salobj.ExpectedError(f"One or more target positions {curr_pos} not in range "
                                           f"{self.pmin_cmd} to {self.pmax_cmd} at the current time")
            if np.any(np.abs(vel) > self.vmax[0:4]):
                raise salobj.ExpectedError(f"Magnitude of one or more target velocities "
                                           f"{vel} > {self.vmax}")
        except Exception as e:
            self.evt_errorCode.set_put(errorCode=1, errorReport=f"trackTarget failed: {e}", force_output=True)
            self.fault()
            raise

        for i in range(4):
            self.actuators[i].set_cmd(pos=pos[i], vel=vel[i], t=data.time)

        target_fields = ("azimuth", "azimuthVelocity",
                         "elevation", "elevationVelocity",
                         "nasmyth1RotatorAngle", "nasmyth1RotatorAngleVelocity",
                         "nasmyth2RotatorAngle", "nasmyth2RotatorAngleVelocity",
                         "time", "trackId", "tracksys", "radesys")
        evt_kwargs = dict((field, getattr(data, field)) for field in target_fields)
        self.evt_target.set_put(**evt_kwargs, force_output=True)
        self.tel_mount_AzEl_Encoders.set(trackId=data.trackId)
        self.tel_mount_Nasmyth_Encoders.set(trackId=data.trackId)

        self._set_tracking_timer(restart=True)

    def _set_tracking_timer(self, restart):
        """Restart or stop the tracking timer.

        Parameters
        ----------
        restart : `bool`
            If True then start or restart the tracking timer, else stop it.
        """
        self._kill_tracking_timer.cancel()
        if restart:
            self._kill_tracking_timer = asyncio.ensure_future(self.kill_tracking())

    def do_setInstrumentPort(self, data):
        self.assert_enabled("setInstrumentPort")
        if self._tracking_enabled:
            raise salobj.ExpectedError("Cannot setInstrumentPort while tracking is enabled")
        port = data.port
        try:
            m3_port_pos_ind = self._port_info_dict[port][0]
        except IndexError:
            raise salobj.ExpectedError(f"Invalid port={port}")
        try:
            m3_port_pos = self.m3_port_pos[m3_port_pos_ind]
        except IndexError:
            raise RuntimeError(f"Bug! invalid m3_port_pos_ind={m3_port_pos_ind} for port={port}")
        self.evt_m3PortSelected.set_put(selected=port)
        m3actuator = self.actuators[Axis.M3]
        if m3actuator.cmd.p0 == m3_port_pos and self.evt_m3InPosition.data.inPosition:
            # already there; don't do anything
            return
        self.actuators[Axis.M3].set_cmd(pos=m3_port_pos, vel=0, t=curr_tai())
        self._axis_enabled[Axis.NA1] = False
        self._axis_enabled[Axis.NA2] = False
        self.update_events()

    async def do_stopTracking(self, data):
        self.assert_enabled("stopTracking")
        if not self._stop_tracking_task.done():
            raise salobj.ExpectedError("Already stopping")
        self._set_tracking_timer(restart=False)
        self._tracking_enabled = False
        for axis in MainAxes:
            self.actuators[axis].stop()
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
        t0 = curr_tai()
        for axis in Axis:
            actuator = self.actuators[axis]
            if actuator.kind(t0) == path.Kind.Stopped:
                self._axis_enabled[axis] = False
            else:
                already_stopped = False
                actuator.stop()
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
        dt = 0.1 + max_end_time - curr_tai()
        if dt > 0:
            await asyncio.sleep(dt)
        for axis in Axis:
            self._axis_enabled[axis] = False
        asyncio.ensure_future(self._run_update_events())

    async def _finish_stop_tracking(self):
        """Wait for the main axes to stop.
        """
        end_times = [self.actuators[axis].curr[-1].t0 for axis in MainAxes]
        max_end_time = max(end_times)
        dt = 0.1 + max_end_time - curr_tai()
        if dt > 0:
            await asyncio.sleep(dt)
        asyncio.ensure_future(self._run_update_events())

    async def _run_update_events(self):
        """Sleep then run update_events.

        Used to call update_events shortly after the _disable_all_drives_task
        _stop_tracking_task are done.
        """
        await asyncio.sleep(0)
        self.update_events()

    async def implement_simulation_mode(self, simulation_mode):
        if simulation_mode != 1:
            raise salobj.ExpectedError(
                f"This CSC only supports simulation; simulation_mode={simulation_mode} but must be 1")

    def m3_port_rot(self, t):
        """Return exit port and rotator axis.

        Parameters
        ----------
        t : `float`
            Current time, TAI unix seconds.

        Returns
        -------
        port_rot : `tuple`
            Exit port and rotator axis, as a tuple:

            * exit port: an M3ExitPort enum value
            * rotator axis: the instrument rotator at this port,
              as an Axis enum value, or None if the port has no rotator.
        """
        if not self.m3_in_position(t):
            return (None, None)
        cmd_pos = self.actuators[Axis.M3].cmd.p0
        for exit_port, (ind, m3state, rot_axis) in self._port_info_dict.items():
            if self.m3_port_pos[ind] == cmd_pos:
                return (exit_port, rot_axis)
        return (None, None)

    def m3_in_position(self, t):
        """Is the M3 actuator in position?

        Parameters
        ----------
        t : `float`
            Current time, TAI unix seconds.
        """
        m3actuator = self.actuators[Axis.M3]
        if m3actuator.kind(t) != path.Kind.Stopped:
            return False
        m3cmd_pos = m3actuator.cmd.p0
        m3curr_pos, m3curr_vel = m3actuator.curr[-1].pva(t)[0:2]
        return abs(m3cmd_pos - m3curr_pos) < self.m3tolerance

    def report_summary_state(self):
        super().report_summary_state()
        if self.summary_state == salobj.State.ENABLED:
            axes_to_enable = set((Axis.Elevation, Axis.Azimuth))
            t = curr_tai()
            rot_axis = self.m3_port_rot(t)[1]
            if rot_axis is not None:
                axes_to_enable.add(rot_axis)
            for axis in Axis:
                self._axis_enabled[axis] = axis in axes_to_enable
        else:
            self.disable_all_drives()
        if self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            if self._events_and_telemetry_task.done():
                self._events_and_telemetry_task = asyncio.ensure_future(self.events_and_telemetry_loop())
        else:
            self._events_and_telemetry_task.cancel()

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

    def update_events(self):
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
        t = curr_tai()
        curr_pos = np.array([actuator.curr.pva(t)[0] for actuator in self.actuators], dtype=float)
        m3actuator = self.actuators[Axis.M3]
        axes_in_use = set([Axis.Elevation, Axis.Azimuth, Axis.M3])

        # Handle M3 actuator; set_cmd needs to be called to transition
        # from slewing to tracking, and that is done here for M3
        # (the trackPosition command does that for the other axes).
        m3arrived = m3actuator.kind(t) == path.Kind.Slewing and t > m3actuator.curr[-1].t0
        if m3arrived:
            m3actuator.curr = path.Path(path.TPVAJ(t0=t, p0=m3actuator.cmd.p0), kind=path.Kind.Stopped)
        exit_port, rot_axis = self.m3_port_rot(t)
        if rot_axis is not None:
            axes_in_use.add(rot_axis)
            if m3arrived:
                self._axis_enabled[rot_axis] = True

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

        # Handle brakes
        for axis in Axis:
            for brake_name in self._brake_names[axis]:
                self.set_event(brake_name, engaged=not self._axis_enabled[axis])

        # Handle drive status (which means enabled)
        for axis in Axis:
            for evt_name in self._drive_status_names[axis]:
                self.set_event(evt_name, enable=self._axis_enabled[axis])

        # Handle atMountState
        if self._tracking_enabled:
            mount_state = AtMountState.TRACKINGENABLED
        elif not self._stop_tracking_task.done() or not self._disable_all_drives_task.done():
            mount_state = AtMountState.STOPPING
        else:
            mount_state = AtMountState.TRACKINGDISABLED
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
        m3_in_position = self.m3_in_position(t)
        self.evt_m3InPosition.set_put(inPosition=m3_in_position)

        # Handle "in position" events for the main axes.
        # Main axes are in position if enabled
        # and actuator.kind(t) is tracking.
        if not self._tracking_enabled:
            for axis in MainAxes:
                self.set_event(self._in_position_names[axis], inPosition=False)
            self.evt_allAxesInPosition.set_put(inPosition=False)
        else:
            all_in_position = m3_in_position
            for axis in MainAxes:
                if not self._axis_enabled[axis]:
                    in_position = False
                else:
                    kind = self.actuators[axis].kind(t)
                    in_position = kind == path.Kind.Tracking
                if not in_position and axis in axes_in_use:
                    all_in_position = False
                self.set_event(self._in_position_names[axis], inPosition=in_position)
            self.evt_allAxesInPosition.set_put(inPosition=all_in_position)

        # compute m3_state for use setting m3State.state
        # and m3RotatorDetentSwitches
        m3_state = None
        if m3_in_position:
            # we are either at a port or at an unknown position
            # exit port enum values = m3state enum values
            # for the known exit ports
            if exit_port is not None:
                m3_state = exit_port
            else:
                # Move is finished, but not at a known point
                m3_state = M3State.UNKNOWNPOSITION
        elif m3actuator.kind(t) == path.Kind.Slewing:
            m3_state = M3State.INMOTION
        else:
            m3_state = M3State.UNKNOWNPOSITION
        assert m3_state is not None

        # handle m3State
        self.evt_m3State.set_put(state=m3_state)

        # Handle M3 detent switch
        detent_map = {
            1: "nasmyth1Active",
            2: "nasmyth2Active",
            3: "port3Active",
        }
        at_field = detent_map.get(m3_state, None)
        detent_values = dict((field_name, field_name == at_field) for field_name in detent_map.values())
        self.evt_m3RotatorDetentSwitches.set_put(**detent_values)

    def update_telemetry(self):
        """Output all telemetry topics.
        """
        nitems = len(self.tel_mount_AzEl_Encoders.data.elevationEncoder1Raw)
        curr_time = curr_tai()

        times = np.linspace(start=curr_time - self._telemetry_interval,
                            stop=curr_time,
                            num=nitems, endpoint=True)

        for i, t in enumerate(times):
            pva_list = [actuator.curr.pva(t) for actuator in self.actuators]
            curr_pos = np.array([pva[0] for pva in pva_list], dtype=float)
            curr_vel = np.array([pva[1] for pva in pva_list], dtype=float)
            curr_accel = np.array([pva[2] for pva in pva_list], dtype=float)

            axis_encoder_counts = (curr_pos*self.axis_encoder_counts_per_deg).astype(int)
            torque = curr_accel*self.torque_per_accel
            motor_pos = curr_pos * self.motor_axis_ratio
            motor_pos = (motor_pos + 360) % 360 - 360
            motor_encoder_counts = (motor_pos*self.motor_encoder_counts_per_deg).astype(int)

            trajectory_data = self.tel_trajectory.data
            trajectory_data.elevation[i] = curr_pos[Axis.Elevation]
            trajectory_data.azimuth[i] = curr_pos[Axis.Azimuth]
            trajectory_data.nasmyth1RotatorAngle[i] = curr_pos[Axis.NA1]
            trajectory_data.nasmyth2RotatorAngle[i] = curr_pos[Axis.NA2]
            trajectory_data.elevationVelocity[i] = curr_vel[Axis.Elevation]
            trajectory_data.azimuthVelocity[i] = curr_vel[Axis.Azimuth]
            trajectory_data.nasmyth1RotatorAngleVelocity[i] = curr_vel[Axis.NA1]
            trajectory_data.nasmyth2RotatorAngleVelocity[i] = curr_vel[Axis.NA2]

            azel_encoders_data = self.tel_mount_AzEl_Encoders.data
            azel_encoders_data.elevationCalculatedAngle[i] = curr_pos[Axis.Elevation]
            azel_encoders_data.elevationEncoder1Raw[i] = axis_encoder_counts[Axis.Elevation]
            azel_encoders_data.elevationEncoder2Raw[i] = axis_encoder_counts[Axis.Elevation]
            azel_encoders_data.elevationEncoder3Raw[i] = axis_encoder_counts[Axis.Elevation]
            azel_encoders_data.azimuthCalculatedAngle[i] = curr_pos[Axis.Azimuth]
            azel_encoders_data.azimuthEncoder1Raw[i] = axis_encoder_counts[Axis.Azimuth]
            azel_encoders_data.azimuthEncoder2Raw[i] = axis_encoder_counts[Axis.Azimuth]
            azel_encoders_data.azimuthEncoder3Raw[i] = axis_encoder_counts[Axis.Azimuth]

            nasmyth_encoders_data = self.tel_mount_Nasmyth_Encoders.data
            nasmyth_encoders_data.nasmyth1CalculatedAngle[i] = curr_pos[Axis.NA1]
            nasmyth_encoders_data.nasmyth1Encoder1Raw[i] = axis_encoder_counts[Axis.NA1]
            nasmyth_encoders_data.nasmyth1Encoder2Raw[i] = axis_encoder_counts[Axis.NA1]
            nasmyth_encoders_data.nasmyth1Encoder3Raw[i] = axis_encoder_counts[Axis.NA1]
            nasmyth_encoders_data.nasmyth2CalculatedAngle[i] = curr_pos[Axis.NA2]
            nasmyth_encoders_data.nasmyth2Encoder1Raw[i] = axis_encoder_counts[Axis.NA2]
            nasmyth_encoders_data.nasmyth2Encoder2Raw[i] = axis_encoder_counts[Axis.NA2]
            nasmyth_encoders_data.nasmyth2Encoder3Raw[i] = axis_encoder_counts[Axis.NA2]

            torqueDemand_data = self.tel_torqueDemand.data
            torqueDemand_data.elevationMotorTorque[i] = torque[Axis.Elevation]
            torqueDemand_data.azimuthMotor1Torque[i] = torque[Axis.Azimuth]
            torqueDemand_data.azimuthMotor2Torque[i] = torque[Axis.Azimuth]
            torqueDemand_data.nasmyth1MotorTorque[i] = torque[Axis.NA1]
            torqueDemand_data.nasmyth2MotorTorque[i] = torque[Axis.NA2]

            measuredTorque_data = self.tel_measuredTorque.data
            measuredTorque_data.elevationMotorTorque[i] = torque[Axis.Elevation]
            measuredTorque_data.azimuthMotor1Torque[i] = torque[Axis.Azimuth]
            measuredTorque_data.azimuthMotor2Torque[i] = torque[Axis.Azimuth]
            measuredTorque_data.nasmyth1MotorTorque[i] = torque[Axis.NA1]
            measuredTorque_data.nasmyth2MotorTorque[i] = torque[Axis.NA2]

            measuredMotorVelocity_data = self.tel_measuredMotorVelocity.data
            measuredMotorVelocity_data.elevationMotorVelocity[i] = curr_vel[Axis.Elevation]
            measuredMotorVelocity_data.azimuthMotor1Velocity[i] = curr_vel[Axis.Azimuth]
            measuredMotorVelocity_data.azimuthMotor2Velocity[i] = curr_vel[Axis.Azimuth]
            measuredMotorVelocity_data.nasmyth1MotorVelocity[i] = curr_vel[Axis.NA1]
            measuredMotorVelocity_data.nasmyth2MotorVelocity[i] = curr_vel[Axis.NA2]

            azel_mountMotorEncoders_data = self.tel_azEl_mountMotorEncoders.data
            azel_mountMotorEncoders_data.elevationEncoder[i] = motor_pos[Axis.Elevation]
            azel_mountMotorEncoders_data.azimuth1Encoder[i] = motor_pos[Axis.Azimuth]
            azel_mountMotorEncoders_data.azimuth2Encoder[i] = motor_pos[Axis.Azimuth]
            azel_mountMotorEncoders_data.elevationEncoderRaw[i] = motor_encoder_counts[Axis.Elevation]
            azel_mountMotorEncoders_data.azimuth1EncoderRaw[i] = motor_encoder_counts[Axis.Azimuth]
            azel_mountMotorEncoders_data.azimuth2EncoderRaw[i] = motor_encoder_counts[Axis.Azimuth]

            nasmyth_m3_mountMotorEncoders_data = self.tel_nasymth_m3_mountMotorEncoders.data
            nasmyth_m3_mountMotorEncoders_data.nasmyth1Encoder[i] = motor_pos[Axis.NA1]
            nasmyth_m3_mountMotorEncoders_data.nasmyth2Encoder[i] = motor_pos[Axis.NA2]
            nasmyth_m3_mountMotorEncoders_data.m3Encoder[i] = motor_pos[Axis.M3]
            nasmyth_m3_mountMotorEncoders_data.nasmyth1EncoderRaw[i] = motor_encoder_counts[Axis.NA1]
            nasmyth_m3_mountMotorEncoders_data.nasmyth2EncoderRaw[i] = motor_encoder_counts[Axis.NA2]
            nasmyth_m3_mountMotorEncoders_data.m3EncoderRaw[i] = motor_encoder_counts[Axis.M3]

        self.tel_trajectory.set_put(cRIO_timestamp=times[0])
        self.tel_mount_AzEl_Encoders.set_put(cRIO_timestamp=times[0])
        self.tel_mount_Nasmyth_Encoders.set_put(cRIO_timestamp=times[0])
        self.tel_torqueDemand.set_put(cRIO_timestamp=times[0])
        self.tel_measuredTorque.set_put(cRIO_timestamp=times[0])
        self.tel_measuredMotorVelocity.set_put(cRIO_timestamp=times[0])
        self.tel_azEl_mountMotorEncoders.set_put(cRIO_timestamp=times[0])
        self.tel_nasymth_m3_mountMotorEncoders.set_put(cRIO_timestamp=times[0])

    async def events_and_telemetry_loop(self):
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
        i = 0
        while self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            # update events first so that limits are handled
            i += 1
            self.update_events()

            if i >= self._events_per_telemetry:
                i = 0
                self.update_telemetry()

            await asyncio.sleep(self._telemetry_interval/self._events_per_telemetry)
