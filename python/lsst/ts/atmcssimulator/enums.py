# This file is part of ts_atmcssimulator.
#
# # Developed for the Vera C. Rubin Observatory Telescope and Site Systems.
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

__all__ = [
    "Axis",
    "Command",
    "CommandArgument",
    "Event",
    "MainAxes",
    "Telemetry",
]

import enum


class Axis(enum.IntEnum):
    Elevation = 0
    Azimuth = 1
    NA1 = 2
    NA2 = 3
    M3 = 4


class Command(str, enum.Enum):
    """Enum containing all command names."""

    SET_INSTRUMENT_PORT = "cmd_setInstrumentPort"
    START_TRACKING = "cmd_startTracking"
    STOP_TRACKING = "cmd_stopTracking"
    TRACK_TARGET = "cmd_trackTarget"


class CommandArgument(str, enum.Enum):
    """Enum containing all possible command arguments."""

    AZIMUTH = "azimuth"
    AZIMUTH_VELOCITY = "azimuthVelocity"
    ELEVATION = "elevation"
    ELEVATION_VELOCITY = "elevationVelocity"
    NASMYTH1_ROTATOR_ANGLE = "nasmyth1RotatorAngle"
    NASMYTH1_ROTATOR_ANGLE_VELOCITY = "nasmyth1RotatorAngleVelocity"
    NASMYTH2_ROTATOR_ANGLE = "nasmyth2RotatorAngle"
    NASMYTH2_ROTATOR_ANGLE_VELOCITY = "nasmyth2RotatorAngleVelocity"
    PORT = "port"
    RA_DE_SYS = "radesys"
    TAI_TIME = "taiTime"
    TRACK_ID = "trackId"
    TRACK_SYS = "tracksys"


class Event(str, enum.Enum):
    """Enum containing all event names."""

    ALLAXESINPOSITION = "evt_allAxesInPosition"
    ATMOUNTSTATE = "evt_atMountState"
    AZIMUTHBRAKE1 = "evt_azimuthBrake1"
    AZIMUTHBRAKE2 = "evt_azimuthBrake2"
    AZIMUTHDRIVE1STATUS = "evt_azimuthDrive1Status"
    AZIMUTHDRIVE2STATUS = "evt_azimuthDrive2Status"
    AZIMUTHINPOSITION = "evt_azimuthInPosition"
    AZIMUTHLIMITSWITCHCCW = "evt_azimuthLimitSwitchCCW"
    AZIMUTHLIMITSWITCHCW = "evt_azimuthLimitSwitchCW"
    AZIMUTHTOPPLEBLOCKCCW = "evt_azimuthToppleBlockCCW"
    AZIMUTHTOPPLEBLOCKCW = "evt_azimuthToppleBlockCW"
    DETAILEDSTATE = "evt_detailedState"
    ELEVATIONBRAKE = "evt_elevationBrake"
    ELEVATIONDRIVESTATUS = "evt_elevationDriveStatus"
    ELEVATIONINPOSITION = "evt_elevationInPosition"
    ELEVATIONLIMITSWITCHLOWER = "evt_elevationLimitSwitchLower"
    ELEVATIONLIMITSWITCHUPPER = "evt_elevationLimitSwitchUpper"
    M3DRIVESTATUS = "evt_m3DriveStatus"
    M3INPOSITION = "evt_m3InPosition"
    M3PORTSELECTED = "evt_m3PortSelected"
    M3ROTATORDETENTSWITCHES = "evt_m3RotatorDetentSwitches"
    M3ROTATORLIMITSWITCHCCW = "evt_m3RotatorLimitSwitchCCW"
    M3ROTATORLIMITSWITCHCW = "evt_m3RotatorLimitSwitchCW"
    M3STATE = "evt_m3State"
    NASMYTH1BRAKE = "evt_nasmyth1Brake"
    NASMYTH1DRIVESTATUS = "evt_nasmyth1DriveStatus"
    NASMYTH1LIMITSWITCHCCW = "evt_nasmyth1LimitSwitchCCW"
    NASMYTH1LIMITSWITCHCW = "evt_nasmyth1LimitSwitchCW"
    NASMYTH1ROTATORINPOSITION = "evt_nasmyth1RotatorInPosition"
    NASMYTH2BRAKE = "evt_nasmyth2Brake"
    NASMYTH2DRIVESTATUS = "evt_nasmyth2DriveStatus"
    NASMYTH2LIMITSWITCHCCW = "evt_nasmyth2LimitSwitchCCW"
    NASMYTH2LIMITSWITCHCW = "evt_nasmyth2LimitSwitchCW"
    NASMYTH2ROTATORINPOSITION = "evt_nasmyth2RotatorInPosition"
    POSITIONLIMITS = "evt_positionLimits"
    TARGET = "evt_target"


MainAxes = (Axis.Elevation, Axis.Azimuth, Axis.NA1, Axis.NA2)


class Telemetry(str, enum.Enum):
    """Enum containing all telemetry names."""

    AZEL_MOTOR_MOUNT_ENCODERS = "tel_azEl_mountMotorEncoders"
    MEASURED_MOTOR_VELOCITY = "tel_measuredMotorVelocity"
    MEASURED_TORQUE = "tel_measuredTorque"
    MOUNT_AZEL_ENCODERS = "tel_mount_AzEl_Encoders"
    MOUNT_NASMYTH_ENCODERS = "tel_mount_Nasmyth_Encoders"
    NASMYTH_M3_MOUNT_MOTOR_ENCODERS = "tel_nasymth_m3_mountMotorEncoders"
    TORQUE_DEMAND = "tel_torqueDemand"
    TRAJECTORY = "tel_trajectory"
