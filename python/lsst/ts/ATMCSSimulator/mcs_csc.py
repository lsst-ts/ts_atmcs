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

__all__ = ["ATMCSCsc"]

import asyncio
import enum
import time

import numpy as np

from lsst.ts import salobj
from .actuator import Actuator

import SALPY_ATMCS


class Axis(enum.IntEnum):
    Azimuth = 0
    Elevation = 1
    NA1 = 2
    NA2 = 3
    M3 = 4


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
    - azimuth
    - elevation
    - Nasmyth1 rotator
    - Nasmyth2 rotator
    - m3 rotator

    **Limitations**

    * Jerk is infinite.
    * The two azimuth motors are identical.
    * The only way to hit a limit switch is to configure the position
      of that switch within the command limits for that axis.
    * The CSC always wakes up at position 0 for all axes.
    * The model for the azimuth topple block is primitive:

      * CCW active if az < az1
      * CW active if az > az2
      * neither active otherwise
    * The path generator is a bit primitive in that it always computes
      a trapezoidal slew, even while tracking. In practice the resulting
      path is probably going to be perfectly usable.
    * The home position is always the minimum commanded position;
      this would be easy to change if desired.
    """
    def __init__(self, initial_state=salobj.State.STANDBY, initial_simulation_mode=1):
        super().__init__(SALPY_ATMCS, index=0, initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)
        self.telemetry_interval = 0.2  # seconds
        self.stop_gently_task = None

        # Event topics are:
        # m3State_data
        # elevationInPosition_data
        # azimuthInPosition_data
        # nasmyth1RotatorInPosition_data
        # nasmyth2RotatorInPosition_data
        # m3InPosition_data
        # allAxesInPosition_data
        # azimuthLimitSwitchCCW_data
        # elevationLimitSwitchUpper_data
        # nasmyth1LimitSwitchCW_data
        # nasmyth2LimitSwitchCCW_data
        # azimuthBrake1_data
        # azimuthBrake2_data
        # elevationBrake_data
        # nasmyth1Brake_data
        # nasmyth2Brake_data
        # azimuthToppleBlockCCW_data
        # nasmyth1LimitSwitchCCW_data
        # azimuthToppleBockCW_data
        # nasmyth2LimitSwitchCW_data
        # azimuthLimitSwitchCW_data
        # azimuthDrive1Status_data
        # azimuthDrive2Status_data
        # elevationDriveStatus_data
        # nasmyth1DriveStatus_data
        # nasmyth2DriveStatus_data
        # m3DriveStatus_data
        # elevationLimitsSwitchLower_data
        # atMountState_data
        # m3RotatorLimitSwitchCW_data
        # m3RotatorLimitSwitchCCW_data
        # m3RotatorDetentLimitSwitch_data
        # m3PortSelected_data

        # data for telemetry topics
        self.mountEncoders_data = self.tel_mountEncoders.DataType()
        self.torqueDemand_data = self.tel_torqueDemand.DataType()
        self.measuredTorque_data = self.tel_measuredTorque.DataType()
        self.measuredMotorVelocity_data = self.tel_measuredMotorVelocity.DataType()
        self.mountMotorEncoders_data = self.tel_mountMotorEncoders.DataType()
        """Index of rotator for current index port, or None of no rotator.

        Values:

        * NA1: 2
        * NA2: 3
        * other: None
        """
        self.port_info_dict = {
            SALPY_ATMCS.ATMCS_shared_M3ExitPort_Nasmyth1: 0,
            SALPY_ATMCS.ATMCS_shared_M3ExitPort_Nasmyth2: 1,
            SALPY_ATMCS.ATMCS_shared_M3ExitPort_Port3: 2,
        }
        """Dict of port enum: port_az index"""

        self.min_lim_names = (
            "azimuthLimitSwitchCW",
            "elevationLimitSwitchLower",
            "nasmyth1LimitSwitchCW",
            "nasmyth2LimitSwitchCW",
            "m3RotatorLimitSwitchCW",
        )
        """Name of minimum limit switch event for each axis."""

        self.max_lim_names = (
            "azimuthLimitSwitchCCW",
            "elevationLimitSwitchUpper",
            "nasmyth1LimitSwitchCCW",
            "nasmyth2LimitSwitchCCW",
            "m3RotatorLimitSwitchCCW",
        )
        """Name of maximum limit switch event for each axis."""

        self.in_position_names = (
            "azimuthInPosition",
            "elevationInPosition",
            "nasmyth1RotatorInPosition",
            "nasmyth2RotatorInPosition",
            "m3InPosition",
        )
        """Name of "in position" event for each individual axis;
        excludes ``allAxesInPosition``.
        """

        self.drive_status_names = (
            "azimuthDrive1Status",
            "azimuthDrive2Status",
            "elevationDriveStatus",
            "nasmyth1DriveStatus",
            "nasmyth2DriveStatus",
            "m3DriveStatus",
        )
        """Name of status for each drive motor."""

        self.brake_names = (
            "azimuthBrake1",
            "azimuthBrake2",
            "elevationBrake",
            "nasmyth1Brake",
            "nasmyth2Brake",
        )
        """Name of brake for each drive motor.

        Warning: there is no brake for M3.
        """

        self.drive_indices = {
            Axis.Azimuth: (0, 1),
            Axis.Elevation: (2,),
            Axis.NA1: (3,),
            Axis.NA2: (4,),
            Axis.M3: (5,),
        }
        """Dict of axis: tuple of one or more drive motor indices."""

        self.tracking_enabled = False
        """Has tracking been enabled by startTracking?

        This remains true until stopTracking is called or the
        summary state is no longer salobj.State.Enabled,
        even if some drives have been disabled by running into limits.
        """

        self.axis_enabled = np.zeros([5], dtype=bool)

        self.axis_braked = np.zeros([5], dtype=bool)
        """If True the axis has its brakes (if any) applied

        In this state axis_enabled should be False
        and the actuator should be halted.
        """

        self.in_position_counts = np.zeros(5, dtype=int)
        """Number of consecutive tests where each axis was in position.

        This should be updated every time telemetry is output,
        and if the count is > needed_in_pos then stop it there.
        """

        self._initialized = False
        self.initialize()
        self.configure()

    def drive_status_names_per_actuator(self, axis):
        axis = Axis(axis)
        if axis == 0:
            return self.drive_status_names[0], self.drive_status_names[1]
        else:
            return self.drive_status_names[axis+1]

    def brake_names_per_actuator(self, axis):
        axis = Axis(axis)
        if axis == 0:
            return self.brake_names[0], self.brake_names[1]
        else:
            return self.brake_names[axis+1]

    def configure(self,
                  min_cmd=(-180, 5, -360, -360, 0),
                  max_cmd=(360, 90, 360, 360, 180),
                  min_lim=(-182, 3, -362, -362, -2),
                  max_lim=(362, 92, 362, 362, 182),
                  max_vel=(5, 5, 5, 5, 5),
                  max_accel=(3, 3, 3, 3, 3),
                  max_in_position_err=(0.01, 0.01, 0.01, 0.01, 0.01),
                  min_in_position_num=3,
                  topple_az=(2, 5),
                  port_az=(0, 90, 180),
                  m3_detent_range=0.01,
                  needed_in_pos=3,
                  ):
        """Set configuration.

        Parameters
        ----------
        min_cmd : ``iterable`` of 5 `float`
            Minimum commanded position for each axis_, in deg
        max_cmd : ``iterable`` of 5 `float`
            Minimum commanded position for each axis_, in deg
        min_lim : ``iterable`` of 5 `float`
            Position of minimum limit switch for each axis_, in deg
        max_lim : ``iterable`` of 5 `float`
            Position of maximum limit switch for each axis_, in deg
        max_vel : ``iterable`` of 5 `float`
            Maximum velocity of each axis, in deg/sec
        max_accel : ``iterable`` of 5 `float`
            Maximum acceleration of each axis, in deg/sec
        max_in_position_err : ``iterable`` of 5 `float`
            Maximum allowed error for each axis to be in position (deg)
        min_in_position_num : `int`
            Minimum number of consecutive times an axis must be in position
            before being reported as such.
        topple_az : ``iterable`` of 2 `float`
            Min, max azimuth at which the topple block moves, in deg
        port_az : ``iterable`` of 3 `float`
            Position of each instrument port: NA1, NA2 and Port3
        m3_detent_range : `float`
            Maximum error beyond which the M3 detent disengages (deg)
        """
        def convert_values(name, values, nval):
            out = np.array(values, dtype=float)
            if out.shape != (nval,):
                raise salobj.ExpectedError(f"Could not format {name}={values!r} as {nval} floats")
            return out

        # convert and check all values first,
        # so nothing changes if any input is invalid
        min_cmd = convert_values(min_cmd, 5)
        max_cmd = convert_values(max_cmd, 5)
        min_lim = convert_values(min_lim, 5)
        max_lim = convert_values(max_lim, 5)
        max_vel = convert_values(max_vel, 5)
        max_accel = convert_values(max_accel, 5)
        max_in_position_err = convert_values(max_in_position_err, 5)
        min_in_position_num = int(min_in_position_num)
        if max_vel.min() <= 0:
            raise salobj.ExpectedError(f"max_vel={max_vel}; all values must be positive")
        if max_accel.min() <= 0:
            raise salobj.ExpectedError(f"max_accel={max_accel}; all values must be positive")
        if max_in_position_err.min() <= 0:
            raise salobj.ExpectedError(f"max_in_position_err={max_in_position_err}; "
                                       "all values must be positive")
        if min_in_position_num <= 0:
            raise salobj.ExpectedError(f"min_in_position_num={min_in_position_num}; must be positive")
        topple_az = convert_values(topple_az, 2)
        port_az = convert_values(port_az, 3)
        m3_detent_range = float(m3_detent_range)
        if m3_detent_range <= 0:
            raise salobj.ExpectedError(f"m3_detent_range={m3_detent_range} must be positive")

        self.min_cmd = min_cmd
        self.max_cmd = max_cmd
        self.min_lim = min_lim
        self.max_lim = max_lim
        self.max_vel = max_vel
        self.max_in_position_err = max_in_position_err
        self.min_in_position_num = min_in_position_num
        self.topple_az = topple_az
        self.port_az = port_az
        self.m3_detent_range = m3_detent_range

        self.actuators = [Actuator(vmax=max_vel[axis], amax=max_accel[axis]) for axis in Axis]

    def do_startTracking(self, id_data):
        self.assert_enabled("startTracking")
        if self.stop_gently_task and not self.stop_gently_task.done():
            raise salobj.ExpectedError("stopTracking not finished yet")
        self.enable_tracking()

    def do_trackTarget(self, id_data):
        self.assert_enabled("trackTarget")
        if not self.tracking_enabled:
            raise salobj.ExpectedError("trackTarget disallowed: drives not enabled")
        data = id_data.data
        pos = np.array([data.azimuth, data.elevation,
                        data.nasmyth1RotatorAngle, data.nasmyth2RotatorAngle], dtype=float)
        vel = np.array([data.azimuthVelocity, data.elevationVelociy,
                       data.nasmyth1RotatorAngleVelocity, data.nasmyth2RotatorAngleVelocity], dtype=float)
        if np.any(pos < self.min_cmd[0:4]) or np.any(pos > self.max_cmd[0:4]):
            raise salobj.ExpectedError(f"One or more commanded positions out of range")
        if np.any(np.abs(vel) > self.max_vel[0:4]):
            raise salobj.ExpectedError(f"One or more commanded velocities out of range")
        for i in range(4):
            self.actuators[i].set_cmd(pos=pos[i], vel=vel[i])

    def do_setInstrumentPort(self, id_data):
        self.assert_enabled("setInstrumentPort")
        port = id_data.data.port
        try:
            port_az_ind = self.port_info_dict[port]
        except IndexError:
            raise salobj.ExpectedError(f"Invalid port={port}")
        try:
            port_az = self.port_az[port_az_ind]
        except IndexError:
            raise RuntimeError(f"Bug! invalid port_az_ind={port_az_ind} for port={port}")
        self.actuators[Axis.M3].set_cmd(pos=port_az, vel=0)
        self.set_event_field("m3PortSelected", "selected", port)

    async def do_stopTracking(self, id_data):
        self.assert_enabled("stopTracking")
        if self.stop_gently_task and not self.stop_gently_task.done():
            raise salobj.ExpectedError("Already stopping")
        self.stop_gently_task = self.stop_gently()
        await self.stop_gently_task
        self.stop_gently_task = None

    def initialize(self):
        self.disable_tracking(gently=False)
        self.update_events()
        self._initialized = True

    async def stop_gently(self):
        """Stop the axes (but not M3) gently.

        Stop M3 abruptly.
        """
        try:
            end_times = []
            for actuator in self.actuators[0:4]:
                actuator.stop()
                end_times.append(actuator.curr[-1].t0)
            max_end_time = max(end_times)
            dt = max_end_time - time.time()
            if dt > 0:
                await asyncio.sleep(dt)
        finally:
            self.disable_tracking(gently=True)

    def update_events(self):
        """Set state of the various events.

        Includes state of drives, brakes, limit switches, azimuth topple block,
        M3 detents and "in position".

        Does *not* include ``m3PortSelected`` because that only needs
        to be set by `do_setInstrumentPort`.

        Report events that have changed,
        and for axes that have run into a limit switch, abort the axis,
        disable its drives and set its brakes.
        """
        t = time.time()
        curr_pos = [actuator.curr.pva(t)[0] for actuator in self.actuators]

        # Handle limit switches
        # including aborting axes that are out of limits
        # and putting on their brakes (if any)
        abort_axes = []
        for axis in Axis:
            self.set_event_field(self.min_lim_names[axis], "active", curr_pos[axis] < self.min_lim[axis])
            self.set_event_field(self.max_lim_names[axis], "active", curr_pos[axis] > self.max_lim[axis])
            if curr_pos[axis] < self.min_lim[axis] or curr_pos[axis] > self.max_lim[axis]:
                abort_axes.append(axis)
        for axis in abort_axes:
            self.actuators[axis].abort()
            self.axis_enabled[axis] = False
            self.axis_braked[axis] = True

        # Handle brakes
        for axis in Axis[0:-1]:  # omit M3 which has no brake
            for drive_index in self.drive_indices(axis):
                for brake_name in self.brake_names(drive_index):
                    self.set_event_field(brake_name, "engage", self.axis_braked[axis])

        # Handle atMountState
        if self.tracking_enabled:
            mount_state = SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabledState
        else:
            curr_vel = np.array([actuator.curr.pva(t)[1] for actuator in self.actuators])
            if all(curr_vel == 0.0):
                mount_state = SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabledState
            else:
                mount_state = SALPY_ATMCS.ATMCS_shared_AtMountState_StoppingState
        self.set_event_field("atMountState", "state", mount_state)

        # Handle azimuth topple block
        if curr_pos[Axis.Azimuth] < self.topple_az[0]:
            self.set_event_field("azimuthToppleBockCCW", "active", True)
            self.set_event_field("azimuthToppleBockCW", "active", False)
        elif curr_pos[Axis.Azimuth] > self.topple_az[1]:
            self.set_event_field("azimuthToppleBockCCW", "active", False)
            self.set_event_field("azimuthToppleBockCW", "active", True)
        else:
            self.set_event_field("azimuthToppleBockCCW", "active", False)
            self.set_event_field("azimuthToppleBockCW", "active", False)

        # Handle M3 detent switch; field order must match index order in port_az
        detent_fields = ("nasmyth1DetentActive", "nasmyth2DetentActive", "port3DetentActive")
        at_port = [abs(curr_pos[Axis.M3] - az) < self.m3_detent_range for az in self.port_az]
        detent_dict = dict((detent_fields[i], at_port[i]) for i in range(3))
        self.set_event_fields("m3RotatorDetentLimitSwitch", **detent_dict)

        # Handle "in position" events, including ``allAxesInPosition``
        if not self.tracking_enabled:
            self.set_event_field("allAxesInPosition", "inPosition", False)
        else:
            for axis in Axis:
                if self.axis_braked[axis] or not self.axis_enabled[axis]:
                    in_position = False
                else:
                    cmd_pos = self.actuators[axis].cmd.pva(t)[0]
                    in_position = abs(curr_pos[axis] - cmd_pos) < self.max_in_position_err
                if in_position:
                    self.in_position_counts[axis] += 1
                    if self.in_position_counts[axis] >= self.min_in_position_num:
                        self.set_event_field(self.in_position_names[axis], "inPosition", True)
                        self.in_position_counts[axis] = self.min_in_position_num
                else:
                    self.in_position_counts[axis] -= 1
                    if self.in_position_counts[axis] <= 0:
                        self.set_event_field(self.in_position_names[axis], "inPosition", False)
                        self.in_position_counts[axis] = 0
            all_axes_in_position = all([getattr(f"evt_{name}", "inPosition")
                                       for name in self.in_position_names])
            self.set_event_field("allAxesInPosition", "inPosition", all_axes_in_position)

    def disable_tracking(self, gently):
        """Disable tracking.

        Parameters
        ----------
        gently : `bool`
            If True then stop the axis.
            If False then abort the axis and apply the brake.
        """
        for axis in Axis:
            self.axis_enabled[axis] = False
            if gently:
                self.actuators[axis].stop()
            else:
                self.actuators[axis].abort()
                self.axis_braked[axis] = True
        self.tracking_enabled = False
        self.update_events()

    def enable_tracking(self):
        """Enable tracking.

        Enable or disable all drives and set ``tracking_enabled``.

        Parameters
        ----------
        enable : `bool`
            If True then enable all axes.
            If False then disable all axes.
        """
        for axis in Axis:
            self.axis_enabled[axis] = True
            self.axis_braked[axis] = False
        self.tracking_enabled = True
        self.update_events()

    def set_event_field(self, evt_name, field_name, value):
        """Set a field for any event and output it if changed
        or not self._initialized

        Parameters
        ----------
        evt_name : `str`
            Event name
        field_name : `str`
            Field name
        value : `any`
            Value for the specified value
        """
        self.set_event_fields(evt_name, **{field_name: value})

    def set_event_fields(self, evt_name, **kwargs):
        evt = self.getattr(f"evt_{evt_name}")
        force_put = not self._initialized
        evt.set_fields(force_put=force_put, **kwargs)

    def report_summary_state(self):
        super().report_summary_state()
        if self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            asyncio.ensure_future(self.telemetry_loop())
        if self.summary_state != salobj.State.ENABLED:
            for axis in Axis:
                self.axis_braked[axis] = True
            self.disable_tracking(gently=False)

    async def telemetry_loop(self):
        while self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            self.update_events()
            self.tel_mountEncoders.put(self.mountEncoders_data)
            self.tel_torqueDemand.put(self.torqueDemandData)
            self.tel_measuredTorque.put(self.measuredTorqueData)
            self.tel_measuredMotorVelocity.put(self.measuredMotorVelocity_data)
            self.tel_mountMotorEncoders.put(self.mountMotorEncoders_data)
            await asyncio.sleep(self.telemetry_interval)
