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

__all__ = ["Ack", "Axis", "CommandKey", "Event", "MainAxes", "Telemetry"]

import enum


class Ack(str, enum.Enum):
    ACK = "ack"
    FAIL = "fail"
    NOACK = "noack"
    SUCCESS = "success"


class Axis(enum.IntEnum):
    Elevation = 0
    Azimuth = 1
    NA1 = 2
    NA2 = 3
    M3 = 4


class CommandKey(str, enum.Enum):
    AZIMUTH = "azimuth"
    AZIMUTH_VELOCITY = "azimuthVelocity"
    ELEVATION = "elevation"
    ELEVATION_VELOCITY = "elevationVelocity"
    ID = "id"
    NASMYTH1_ROTATOR_ANGLE = "nasmyth1RotatorAngle"
    NASMYTH1_ROTATOR_ANGLE_VELOCITY = "nasmyth1RotatorAngleVelocity"
    NASMYTH2_ROTATOR_ANGLE = "nasmyth2RotatorAngle"
    NASMYTH2_ROTATOR_ANGLE_VELOCITY = "nasmyth2RotatorAngleVelocity"
    PORT = "port"
    RA_DE_SYS = "radesys"
    SEQUENCE_ID = "sequence_id"
    TAI_TIME = "taiTime"
    TRACK_ID = "trackId"
    TRACK_SYS = "tracksys"
    VALUE = "value"


class Event(str, enum.Enum):
    ALLAXESINPOSITION = "allAxesInPosition"
    ATMOUNTSTATE = "atMountState"
    AZIMUTHBRAKE1 = "azimuthBrake1"
    AZIMUTHBRAKE2 = "azimuthBrake2"
    AZIMUTHDRIVE1STATUS = "azimuthDrive1Status"
    AZIMUTHDRIVE2STATUS = "azimuthDrive2Status"
    AZIMUTHINPOSITION = "azimuthInPosition"
    AZIMUTHLIMITSWITCHCCW = "azimuthLimitSwitchCCW"
    AZIMUTHLIMITSWITCHCW = "azimuthLimitSwitchCW"
    AZIMUTHTOPPLEBLOCKCCW = "azimuthToppleBlockCCW"
    AZIMUTHTOPPLEBLOCKCW = "azimuthToppleBlockCW"
    DETAILEDSTATE = "detailedState"
    ELEVATIONBRAKE = "elevationBrake"
    ELEVATIONDRIVESTATUS = "elevationDriveStatus"
    ELEVATIONINPOSITION = "elevationInPosition"
    ELEVATIONLIMITSWITCHLOWER = "elevationLimitSwitchLower"
    ELEVATIONLIMITSWITCHUPPER = "elevationLimitSwitchUpper"
    M3DRIVESTATUS = "m3DriveStatus"
    M3INPOSITION = "m3InPosition"
    M3PORTSELECTED = "m3PortSelected"
    M3ROTATORDETENTSWITCHES = "m3RotatorDetentSwitches"
    M3ROTATORLIMITSWITCHCCW = "m3RotatorLimitSwitchCCW"
    M3ROTATORLIMITSWITCHCW = "m3RotatorLimitSwitchCW"
    M3STATE = "m3State"
    NASMYTH1BRAKE = "nasmyth1Brake"
    NASMYTH1DRIVESTATUS = "nasmyth1DriveStatus"
    NASMYTH1LIMITSWITCHCCW = "nasmyth1LimitSwitchCCW"
    NASMYTH1LIMITSWITCHCW = "nasmyth1LimitSwitchCW"
    NASMYTH1ROTATORINPOSITION = "nasmyth1RotatorInPosition"
    NASMYTH2BRAKE = "nasmyth2Brake"
    NASMYTH2DRIVESTATUS = "nasmyth2DriveStatus"
    NASMYTH2LIMITSWITCHCCW = "nasmyth2LimitSwitchCCW"
    NASMYTH2LIMITSWITCHCW = "nasmyth2LimitSwitchCW"
    NASMYTH2ROTATORINPOSITION = "nasmyth2RotatorInPosition"
    POSITIONLIMITS = "positionLimits"
    TARGET = "target"


MainAxes = (Axis.Elevation, Axis.Azimuth, Axis.NA1, Axis.NA2)


class Telemetry(str, enum.Enum):
    AZEL_MOTOR_MOUNT_ENCODERS = "azEl_mountMotorEncoders"
    MEASURED_MOTOR_VELOCITY = "measuredMotorVelocity"
    MEASURED_TORQUE = "measuredTorque"
    MOUNT_AZEL_ENCODERS = "mount_AzEl_Encoders"
    MOUNT_NASMYTH_ENCODERS = "mount_Nasmyth_Encoders"
    NASMYTH_M3_MOUNT_MOTOR_ENCODERS = "nasymth_m3_mountMotorEncoders"
    TORQUE_DEMAND = "torqueDemand"
    TRAJECTORY = "trajectory"
