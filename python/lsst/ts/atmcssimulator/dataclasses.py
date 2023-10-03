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
    "AXIS_EVENT_DICT",
    "PORT_INFO_DICT",
    "AzElMountMotorEncoders",
    "MeasuredMotorVelocity",
    "MeasuredTorque",
    "MountAzElEncoders",
    "MountNasmythEncoders",
    "NasmythM3MountMotorEncoders",
    "PortInfo",
    "TorqueDemand",
    "Trajectory",
]

from dataclasses import dataclass, field

from lsst.ts.xml.enums.ATMCS import M3ExitPort, M3State

from .enums import Axis, Event


def one_hundred_zeros() -> list[float]:
    """Return a list of 100 zeros."""
    return [0.0] * 100


@dataclass
class Timestamp:
    """Base dataclass for dataclasses that contain a timestamp.

    Attributes
    ----------
    cRIO_timestamp : float
        The TAI unix timestamp [sec].
    """

    cRIO_timestamp: float = 0.0


@dataclass
class AzElMountMotorEncoders(Timestamp):
    """Dataclass holding azimuth and elevation mount motor encoder data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    azimuth1Encoder : list[float]
        The azimuth1 encoder angle [deg].
    azimuth1EncoderRaw : list[float]
        The azimuth1 raw encoder angle [deg].
    azimuth2Encoder : list[float]
        The azimuth2 encoder angle [deg].
    azimuth2EncoderRaw : list[float]
        The azimuth2 raw encoder angle [deg].
    elevationEncoder : list[float]
        The elevation encoder angle [deg].
    elevationEncoderRaw : list[float]
        The elevation raw encoder angle [deg].

    """

    azimuth1Encoder: list[float] = field(default_factory=one_hundred_zeros)
    azimuth1EncoderRaw: list[float] = field(default_factory=one_hundred_zeros)
    azimuth2Encoder: list[float] = field(default_factory=one_hundred_zeros)
    azimuth2EncoderRaw: list[float] = field(default_factory=one_hundred_zeros)
    elevationEncoder: list[float] = field(default_factory=one_hundred_zeros)
    elevationEncoderRaw: list[float] = field(default_factory=one_hundred_zeros)


@dataclass
class MeasuredMotorVelocity(Timestamp):
    """Data class holding measured motor velocity data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    azimuthMotor1Velocity : list[float]
        Azimuth motor1 velocity [deg/sec]
    azimuthMotor2Velocity : list[float]
        Azimuth motor2 velocity [deg/sec].
    elevationMotorVelocity : list[float]
        Elevation motor velocity [deg/sec].
    nasmyth1MotorVelocity : list[float]
        Nasmyth1 motor velocity [deg/sec].
    nasmyth2MotorVelocity : list[float]
        Nasmyth2 motor velocity [deg/sec].
    m3Velocity : list[float]
        M3 motor velocity [deg/sec].
    """

    azimuthMotor1Velocity: list[float] = field(default_factory=one_hundred_zeros)
    azimuthMotor2Velocity: list[float] = field(default_factory=one_hundred_zeros)
    elevationMotorVelocity: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1MotorVelocity: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2MotorVelocity: list[float] = field(default_factory=one_hundred_zeros)
    m3Velocity: list[float] = field(default_factory=one_hundred_zeros)


@dataclass
class MeasuredTorque(Timestamp):
    """Dataclass holding measured torque data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    azimuthMotor1Torque : list[float]
        Azimuth motor1 torque [A].
    azimuthMotor2Torque : list[float]
        Azimuth motor2 torque [A].
    elevationMotorTorque : list[float]
        Elevation motor torque [A].
    nasmyth1MotorTorque : list[float]
        Nasmyth1 motor torque [A].
    nasmyth2MotorTorque : list[float]
        Nasmyth2 motor torque [A].
    m3Torque : list[float]
        M3 torque [A].
    """

    azimuthMotor1Torque: list[float] = field(default_factory=one_hundred_zeros)
    azimuthMotor2Torque: list[float] = field(default_factory=one_hundred_zeros)
    elevationMotorTorque: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1MotorTorque: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2MotorTorque: list[float] = field(default_factory=one_hundred_zeros)
    m3Torque: list[float] = field(default_factory=one_hundred_zeros)


@dataclass
class MountAzElEncoders(Timestamp):
    """Dataclass holding mount azimuth and elevation encoder data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    azimuthCalculatedAngle : list[float]
        Azimuth calculated angle [deg].
    azimuthEncoder1Raw : list[float]
        Azimuth encoder1 raw value [deg].
    azimuthEncoder2Raw : list[float]
        Azimuth encoder1 raw value [deg].
    azimuthEncoder3Raw : list[float]
        Azimuth encoder1 raw value [deg].
    elevationCalculatedAngle : list[float]
        Elevation calculated angle [deg].
    elevationEncoder1Raw : list[float]
        Elevation encoder1 raw value [deg].
    elevationEncoder2Raw : list[float]
        Elevation encoder1 raw value [deg].
    elevationEncoder3Raw : list[float]
        Elevation encoder1 raw value [deg].
    """

    azimuthCalculatedAngle: list[float] = field(default_factory=one_hundred_zeros)
    azimuthEncoder1Raw: list[float] = field(default_factory=one_hundred_zeros)
    azimuthEncoder2Raw: list[float] = field(default_factory=one_hundred_zeros)
    azimuthEncoder3Raw: list[float] = field(default_factory=one_hundred_zeros)
    elevationCalculatedAngle: list[float] = field(default_factory=one_hundred_zeros)
    elevationEncoder1Raw: list[float] = field(default_factory=one_hundred_zeros)
    elevationEncoder2Raw: list[float] = field(default_factory=one_hundred_zeros)
    elevationEncoder3Raw: list[float] = field(default_factory=one_hundred_zeros)
    trackId: int = 0


@dataclass
class MountNasmythEncoders(Timestamp):
    """Dataclass holding mount nasmyth encoder data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    nasmyth1CalculatedAngle : list[float]
        Nasmyth1 calculated angle [deg].
    nasmyth1Encoder1Raw : list[float]
        Nasmyth1 encoder1 raw value [deg].
    nasmyth1Encoder2Raw : list[float]
        Nasmyth1 encoder1 raw value [deg].
    nasmyth1Encoder3Raw : list[float]
        Nasmyth1 enccder1 raw value [deg].
    nasmyth2CalculatedAngle : list[float]
        Nasmyth2 calculated angle [deg].
    nasmyth2Encoder1Raw : list[float]
        Nasmyth2 encoder1 raw value [deg].
    nasmyth2Encoder2Raw : list[float]
        Nasmyth2 encoder1 raw value [deg].
    nasmyth2Encoder3Raw : list[float]
        Nasmyth2 encoder1 raw value [deg].
    """

    nasmyth1CalculatedAngle: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1Encoder1Raw: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1Encoder2Raw: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1Encoder3Raw: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2CalculatedAngle: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2Encoder1Raw: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2Encoder2Raw: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2Encoder3Raw: list[float] = field(default_factory=one_hundred_zeros)
    trackId: int = 0


@dataclass
class NasmythM3MountMotorEncoders(Timestamp):
    """Dataclass holding nasmyth mount motor encoders data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    nasmyth1Encoder : list[float]
        Nasmyth1 encoder value [deg].
    nasmyth1EncoderRaw : list[float]
        Nasmyth1 encoder raw value [deg].
    nasmyth2Encoder : list[float]
        Nasmyth2 encoder value [deg].
    nasmyth2EncoderRaw : list[float]
        Nasmyth2 enccder raw value [deg].
    m3Encoder : list[float]
        M3 encoder value [deg].
    m3EncoderRaw : list[float]
        M3 encoder raw value [deg].
    """

    nasmyth1Encoder: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1EncoderRaw: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2Encoder: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2EncoderRaw: list[float] = field(default_factory=one_hundred_zeros)
    m3Encoder: list[float] = field(default_factory=one_hundred_zeros)
    m3EncoderRaw: list[float] = field(default_factory=one_hundred_zeros)


@dataclass
class PortInfo:
    """Dataclass holding port information.

    Attributes
    ----------
    index : int | None
        M3 position for this port.
    m3state : int
        State of M3 axis when pointing to this port.
    axis : int | None
        Rotator axis at this port, or None if this port has no rotator.
    """

    index: int | None
    m3state: int
    axis: int | None


@dataclass
class TorqueDemand(Timestamp):
    """Dataclass holding torque demand data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    azimuthMotor1Torque : list[float]
        Azimuth motor1 torque [A].
    azimuthMotor2Torque : list[float]
        Azimuth motor2 torque [A].
    elevationMotorTorque : list[float]
        Elevation motor torque [A].
    nasmyth1MotorTorque : list[float]
        Nasmyth1 motor torque [A].
    nasmyth2MotorTorque : list[float]
        Nasmyth2 motor torque [A].
    """

    azimuthMotor1Torque: list[float] = field(default_factory=one_hundred_zeros)
    azimuthMotor2Torque: list[float] = field(default_factory=one_hundred_zeros)
    elevationMotorTorque: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1MotorTorque: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2MotorTorque: list[float] = field(default_factory=one_hundred_zeros)


@dataclass
class Trajectory(Timestamp):
    """Dataclass holding trajectory data.

    Each attributed represents 100 values measured starting at the timestamp.

    Attributes
    ----------
    azimuth : list[float]
        Azimuth angle [deg].
    azimuthVelocity : list[float]
        Azimuth velocity [deg/sec].
    elevation : list[float]
        Elevation angle [deg].
    elevationVelocity : list[float]
        Elevation velocity [deg/sec].
    nasmyth1RotatorAngle : list[float]
        Nasmyth1 rotator angle [deg].
    nasmyth1RotatorAngleVelocity : list[float]
        Nasmyth1 rotator velocity [deg/sec].
    nasmyth2RotatorAngle : list[float]
        Nasmyth2 rotator angle [deg].
    nasmyth2RotatorAngleVelocity : list[float]
        Nasmyth2 rotator velocity [deg/sec].
    """

    azimuth: list[float] = field(default_factory=one_hundred_zeros)
    azimuthVelocity: list[float] = field(default_factory=one_hundred_zeros)
    elevation: list[float] = field(default_factory=one_hundred_zeros)
    elevationVelocity: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1RotatorAngle: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth1RotatorAngleVelocity: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2RotatorAngle: list[float] = field(default_factory=one_hundred_zeros)
    nasmyth2RotatorAngleVelocity: list[float] = field(default_factory=one_hundred_zeros)


@dataclass
class AxisEventInformation:
    """Dataclass holding axis event information.

    Attributes
    ----------
    min_lim : `Event`
        The minimum limit switch event.
    max_lim : `Event`
        The maximum limit switch event.
    in_position : `Event`
        The in position event.
    drive_status : `set`[`Event`]
        The set of drive status events.
    brake : `set`[`Event`]
        The set of brake events.
    """

    min_lim: Event
    max_lim: Event
    in_position: Event
    drive_status: set[Event]
    brake: set[Event]


# Dictionary with the AxisEventInformation for all Axes.
AXIS_EVENT_DICT = {
    Axis.Elevation: AxisEventInformation(
        min_lim=Event.ELEVATIONLIMITSWITCHLOWER,
        max_lim=Event.ELEVATIONLIMITSWITCHUPPER,
        in_position=Event.ELEVATIONINPOSITION,
        drive_status={Event.ELEVATIONDRIVESTATUS},
        brake={Event.ELEVATIONBRAKE},
    ),
    Axis.Azimuth: AxisEventInformation(
        min_lim=Event.AZIMUTHLIMITSWITCHCW,
        max_lim=Event.AZIMUTHLIMITSWITCHCCW,
        in_position=Event.AZIMUTHINPOSITION,
        drive_status={
            Event.AZIMUTHDRIVE1STATUS,
            Event.AZIMUTHDRIVE2STATUS,
        },
        brake={Event.AZIMUTHBRAKE1, Event.AZIMUTHBRAKE2},
    ),
    Axis.NA1: AxisEventInformation(
        min_lim=Event.NASMYTH1LIMITSWITCHCW,
        max_lim=Event.NASMYTH1LIMITSWITCHCCW,
        in_position=Event.NASMYTH1ROTATORINPOSITION,
        drive_status={Event.NASMYTH1DRIVESTATUS},
        brake={Event.NASMYTH1BRAKE},
    ),
    Axis.NA2: AxisEventInformation(
        min_lim=Event.NASMYTH2LIMITSWITCHCW,
        max_lim=Event.NASMYTH2LIMITSWITCHCCW,
        in_position=Event.NASMYTH2ROTATORINPOSITION,
        drive_status={Event.NASMYTH2DRIVESTATUS},
        brake={Event.NASMYTH2BRAKE},
    ),
    Axis.M3: AxisEventInformation(
        min_lim=Event.M3ROTATORLIMITSWITCHCW,
        max_lim=Event.M3ROTATORLIMITSWITCHCCW,
        in_position=Event.M3INPOSITION,
        drive_status={Event.M3DRIVESTATUS},
        brake=set(),
    ),
}


PORT_INFO_DICT = {
    M3ExitPort.NASMYTH1: PortInfo(0, M3State.NASMYTH1, Axis.NA1),
    M3ExitPort.NASMYTH2: PortInfo(1, M3State.NASMYTH2, Axis.NA2),
    M3ExitPort.PORT3: PortInfo(2, M3State.PORT3, None),
}
