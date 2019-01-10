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

import numpy as np

from lsst.ts import salobj
from .actuator import Actuator

import SALPY_ATMCS


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

    * Accleration and jerk are infinite.
    * Reported torque is always 0.
    * The two azimuth motors are identical.
    * The only way to hit a limit switch is to configure the position
      of that switch within the command limits for that axis.
    * The CSC always wakes up at position 0 for all axes.
    * The model for the azimuth topple block is primitive:

      * CCW active if az < az1
      * CW active if az > az2
      * neither active otherwise
    """
    def __init__(self, initial_state=salobj.State.STANDBY, initial_simulation_mode=1):
        super().__init__(SALPY_ATMCS, index=0, initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)
        self.telemetry_interval = 0.2  # seconds

        # data for event topics
        self.m3State_data = self.evt_m3State.DataType()
        self.elevationInPosition_data = self.evt_elevationInPosition.DataType()
        self.azimuthInPosition_data = self.evt_azimuthInPosition.DataType()
        self.allAxesInPosition_data = self.evt_allAxesInPosition.DataType()
        self.nasmyth1RotatorInPosition_data = self.evt_nasmyth1RotatorInPosition.DataType()
        self.azimuthLimitSwitchCCW_data = self.evt_azimuthLimitSwitchCCW.DataType()
        self.elevationLimitSwitchUpper_data = self.evt_elevationLimitSwitchUpper.DataType()
        self.nasmyth1LimitSwitchCW_data = self.evt_nasmyth1LimitSwitchCW.DataType()
        self.nasmyth2LimitSwitchCCW_data = self.evt_nasmyth2LimitSwitchCCW.DataType()
        self.azimuthBrake1_data = self.evt_azimuthBrake1.DataType()
        self.elevationBrake_data = self.evt_elevationBrake.DataType()
        self.nasmyth1Brake_data = self.evt_nasmyth1Brake.DataType()
        self.azimuthToppleBlockCCW_data = self.evt_azimuthToppleBlockCCW.DataType()
        self.azimuthBrake2_data = self.evt_azimuthBrake2.DataType()
        self.nasmyth1LimitSwitchCCW_data = self.evt_nasmyth1LimitSwitchCCW.DataType()
        self.azimuthToppleBockCW_data = self.evt_azimuthToppleBlockCW.DataType()
        self.nasmyth2LimitSwitchCW_data = self.evt_nasmyth2LimitSwitchCW.DataType()
        self.azimuthLimitSwitchCW_data = self.evt_azimuthLimitSwitchCW.DataType()
        self.azimuthDrive1Status_data = self.evt_azimuthDrive1Status.DataType()
        self.azimuthDrive2Status_data = self.evt_azimuthDrive2Status.DataType()
        self.elevationDriveStatus_data = self.evt_elevationDriveStatus.DataType()
        self.nasmyth1DriveStatus_data = self.evt_nasmyth1DriveStatus.DataType()
        self.nasmyth2DriveStatus_data = self.evt_nasmyth2DriveStatus.DataType()
        self.m3DriveStatus_data = self.evt_m3DriveStatus.DataType()
        self.nasmyth2RotatorInPosition_data = self.evt_nasmyth2RotatorInPosition.DataType()
        self.elevationLimitsSwitchLower_data = self.evt_elevationLimitSwitchLower.DataType()
        self.atMountState_data = self.evt_atMountState.DataType()
        self.m3RotatorLimitSwitchCW_data = self.evt_m3RotatorLimitSwitchCW.DataType()
        self.m3RotatorLimitSwitchCCW_data = self.evt_m3RotatorLimitSwitchCCW.DataType()
        self.m3RotatorDetentLimitSwitch_data = self.evt_m3RotatorDetentLimitSwitch.DataType()
        self.m3InPosition_data = self.evt_m3InPosition.DataType()
        self.m3PortSelected_data = self.evt_m3PortSelected.DataType()

        # data for telemetry topics
        self.mountEncoders_data = self.tel_mountEncoders.DataType()
        self.torqueDemand_data = self.tel_torqueDemand.DataType()
        self.measuredTorque_data = self.tel_measuredTorque.DataType()
        self.measuredMotorVelocity_data = self.tel_measuredMotorVelocity.DataType()
        self.mountMotorEncoders_data = self.tel_mountMotorEncoders.DataType()

        self._initialized = False
        self.initialize()

        self.configure()

    def configure(self,
                  min_cmd=(-180, 5, -360),
                  max_cmd=(360, 90, 360),
                  min_lim=(-182, 3, 362),
                  max_lim=(362, 92, 362),
                  vel=(5, 5, 5),
                  accel=(3, 3, 3),
                  topple_az=(2, 5),
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
        vel : ``iterable`` of 5 `float`
            Maximum velocity of each axis, in deg/sec
        accel : ``iterable`` of 5 `float`
            Maximum acceleration of each axis, in deg/sec
        topple_az : ``iterable`` of 2 `float`
            Min, max azimuth at which the topple block moves, in deg
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
        vel = convert_values(vel, 5)
        accel = convert_values(accel, 5)
        if vel.min() <= 0:
            raise salobj.ExpectedError(f"vel={vel}; all values must be positive")
        if accel.min() <= 0:
            raise salobj.ExpectedError(f"accel={vel}; all values must be positive")
        topple_az = convert_values(vel, 2)

        self.min_cmd = min_cmd
        self.max_cmd = max_cmd
        self.min_lim = min_lim
        self.max_lim = max_lim
        self.topple_az = topple_az
        self.actuators = [Actuator(vmax=vel[i], amax=accel[i]) for i in range(5)]

    def do_startTracking(self, id_data):
        raise salobj.ExpectedError("Not yet implemented")

    def do_trackTarget(self, id_data):
        raise salobj.ExpectedError("Not yet implemented")

    def do_setInstrumentPort(self, id_data):
        raise salobj.ExpectedError("Not yet implemented")

    def do_stopTracking(self, id_data):
        raise salobj.ExpectedError("Not yet implemented")

    def initialize(self):
        enable = self.summary_state == salobj.State.ENABLED
        self.enable_drives(enable)
        self.update_limit_switch_state()
        self._initialized = True

    def update_limit_switch_state(self):
        """Set state of limit switches, including azimuth topple_ccw_cw block."""
        axis_names = ("azimuth", "altitude", "nasmyth1", "nasmyth2", "m3Rotator")
        in_reverse_limit = np.curr_pos < np.min_lim
        in_forward_limit = np.curr_pos > np.max_lim
        for i, axis in enumerate(axis_names):
            self.set_event_field(f"{axis}LimitSwitchCW", "active", in_forward_limit[i])
            self.set_event_field(f"{axis}LimitSwitchCCW", "active", in_reverse_limit[i])

        if self.curr_pos < self.topple_az[0]:
            self.set_event_field("azimuthToppleBockCCW", "active", True)
            self.set_event_field("azimuthToppleBockCW", "active", False)
        elif self.curr_pos > self.topple_az[1]:
            self.set_event_field("azimuthToppleBockCCW", "active", False)
            self.set_event_field("azimuthToppleBockCW", "active", True)
        else:
            self.set_event_field("azimuthToppleBockCCW", "active", False)
            self.set_event_field("azimuthToppleBockCW", "active", False)

    def enable_drives(self, enable):
        """Enable or disable all drives and toggle brakes accordingly.

        Parameters
        ----------
        enable : `bool`
            If True the enable all axes.
            If False then disable all axes.
        """
        for drive_prefix in ("azimuthDrive1", "azimuthDrive2",
                             "altitudeDrive",
                             "nasmyth1Drive", "nasmyth2Drive",
                             "m3Drive"):
            evt_name = f"{drive_prefix}Status"
            self.set_event_field(evt_name=evt_name, field_name="enable", value=enable)

        for prefix in ("elevation", "nasmyth1"):
            evt_name = f"{prefix}Brake"
            self.set_event_field(evt_name=evt_name, field_name="engage", value=not enable)

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
        data = self.getattr(f"{evt_name}_data")
        evt = self.getattr(f"evt_{evt_name}")
        if not self._initialized or value != getattr(data, field_name):
            setattr(data, field_name, value)
            evt.put(data)

    def report_summary_state(self):
        super().report_summary_state()
        if self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            asyncio.ensure_future(self.telemetry_loop())

    async def telemetry_loop(self):
        while self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED):
            self.tel_mountEncoders.put(self.mountEncoders_data)
            self.tel_torqueDemand.put(self.torqueDemandData)
            self.tel_measuredTorque.put(self.measuredTorqueData)
            self.tel_measuredMotorVelocity.put(self.measuredMotorVelocity_data)
            self.tel_mountMotorEncoders.put(self.mountMotorEncoders_data)
            await asyncio.sleep(self.telemetry_interval)
