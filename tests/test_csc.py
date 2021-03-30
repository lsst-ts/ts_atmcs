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

import asyncio
import unittest

import asynctest

from lsst.ts import salobj
from lsst.ts import simactuators
from lsst.ts import ATMCSSimulator
from lsst.ts.idl.enums.ATMCS import AtMountState, M3ExitPort, M3State

STD_TIMEOUT = 10  # standard timeout, seconds


class CscTestCase(salobj.BaseCscTestCase, asynctest.TestCase):
    def setUp(self):
        self.axis_names = (  # names of axes for trackTarget command
            "elevation",
            "azimuth",
            "nasmyth1RotatorAngle",
            "nasmyth2RotatorAngle",
        )

    @property
    def brake_events(self):
        return (
            self.remote.evt_azimuthBrake1,
            self.remote.evt_azimuthBrake2,
            self.remote.evt_elevationBrake,
            self.remote.evt_nasmyth1Brake,
            self.remote.evt_nasmyth2Brake,
        )

    @property
    def drive_status_events(self):
        return (
            self.remote.evt_elevationDriveStatus,
            self.remote.evt_azimuthDrive1Status,
            self.remote.evt_azimuthDrive2Status,
            self.remote.evt_nasmyth1DriveStatus,
            self.remote.evt_nasmyth2DriveStatus,
            self.remote.evt_m3DriveStatus,
        )

    @property
    def in_position_events(self):
        return (
            self.remote.evt_elevationInPosition,
            self.remote.evt_azimuthInPosition,
            self.remote.evt_nasmyth1RotatorInPosition,
            self.remote.evt_nasmyth2RotatorInPosition,
            self.remote.evt_m3InPosition,
        )

    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return ATMCSSimulator.ATMCSCsc(initial_state=initial_state)

    async def fault_to_enabled(self):
        """Check that the CSC is in FAULT state and enable it.

        Assumes that the FAULT state has not yet been read from the remote.
        """
        await self.assert_next_summary_state(salobj.State.FAULT)

        await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
        await self.assert_next_summary_state(salobj.State.STANDBY)

        await self.remote.cmd_start.start(timeout=STD_TIMEOUT)
        await self.assert_next_summary_state(salobj.State.DISABLED)

        await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
        await self.assert_next_summary_state(salobj.State.ENABLED)

    async def test_initial_info(self):
        """Check that all events and telemetry are output at startup

        except the m3PortSelected event
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_softwareVersions,
                cscVersion=ATMCSSimulator.__version__ + " sim",
                subsystemVersions="",
            )

            for event_name in self.csc.salinfo.event_names:
                if event_name in (
                    "m3PortSelected",  # output by setInstrumentPort
                    "target",  # output by trackTarget
                    "summaryState",  # already read
                    "softwareVersions",  # not yet supported by salobj
                    "authList",  # only present for SAL 4.2 and later
                    "appliedSettingsMatchStart",
                    "detailedState",
                    "errorCode",
                    "logMessage",
                    "settingVersions",
                    "settingsApplied",  # not yet supported by salobj
                ):
                    continue
                with self.subTest(event_name=event_name):
                    event = getattr(self.remote, f"evt_{event_name}")
                    await event.next(flush=False, timeout=STD_TIMEOUT)

            timeout = STD_TIMEOUT
            for tel_name in self.csc.salinfo.telemetry_names:
                with self.subTest(tel_name=tel_name):
                    tel = getattr(self.remote, f"tel_{tel_name}")
                    await tel.next(flush=False, timeout=timeout)
                timeout = 0.1

    async def test_invalid_track_target(self):
        """Test all reasons trackTarget may be rejected."""
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            # Get the initial summary state, so `fault_to_enabled` sees FAULT.
            await self.assert_next_summary_state(salobj.State.ENABLED)

            min_commanded_position = (5, -270, -165, -165, 0)
            max_commanded_position = (90, 270, 165, 165, 180)
            max_velocity = (100,) * 5
            self.csc.configure(
                min_commanded_position=min_commanded_position,
                max_commanded_position=max_commanded_position,
                max_velocity=max_velocity,
                max_acceleration=(200,) * 5,
            )
            good_target_kwargs = dict((name, 0) for name in self.axis_names)
            # elevation does not have 0 in its valid range
            good_target_kwargs[self.axis_names[0]] = min_commanded_position[0]
            # zero velocity as well
            velocity_kwargs = dict((f"{name}Velocity", 0) for name in self.axis_names)
            good_target_kwargs.update(velocity_kwargs)
            good_target_kwargs["trackId"] = 137

            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

            # Cannot send trackTarget while tracking disabled;
            # this error does not change the summary state.
            self.remote.cmd_trackTarget.set(
                taiTime=salobj.current_tai(), **good_target_kwargs
            )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_trackTarget.start(timeout=1)

            # The rejected target should not be output as an event.
            with self.assertRaises(asyncio.TimeoutError):
                await self.remote.evt_target.next(flush=False, timeout=0.1)

            # Enable tracking and try again; this time it should work.
            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
            )
            await self.remote.cmd_trackTarget.start(timeout=1)
            await asyncio.sleep(0.1)

            # Disable tracking.
            await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.STOPPING
            )
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

            for axis in ATMCSSimulator.Axis:
                if axis is ATMCSSimulator.Axis.M3:
                    continue  # trackTarget doesn't accept M3

                with self.subTest(axis=axis):
                    await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                    await self.assert_next_sample(
                        self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
                    )

                    min_position_kwargs = good_target_kwargs.copy()
                    min_position_kwargs[self.axis_names[axis]] = (
                        min_commanded_position[axis] - 0.000001
                    )
                    self.remote.cmd_trackTarget.set(
                        taiTime=salobj.current_tai(), **min_position_kwargs
                    )
                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_trackTarget.start(timeout=1)
                    await self.assert_next_sample(
                        self.remote.evt_atMountState,
                        state=AtMountState.TRACKINGDISABLED,
                    )
                    await self.fault_to_enabled()

                    await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                    await self.assert_next_sample(
                        self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
                    )

                    max_position_kwargs = good_target_kwargs.copy()
                    max_position_kwargs[self.axis_names[axis]] = (
                        max_commanded_position[axis] + 0.000001
                    )
                    self.remote.cmd_trackTarget.set(
                        taiTime=salobj.current_tai(), **max_position_kwargs
                    )
                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_trackTarget.start(timeout=1)
                    await self.assert_next_sample(
                        self.remote.evt_atMountState,
                        state=AtMountState.TRACKINGDISABLED,
                    )
                    await self.fault_to_enabled()

                    await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                    await self.assert_next_sample(
                        self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
                    )

                    max_velocity_kwargs = good_target_kwargs.copy()
                    max_velocity_kwargs[f"{self.axis_names[axis]}Velocity"] = (
                        max_velocity[axis] + 0.000001
                    )
                    self.remote.cmd_trackTarget.set(
                        taiTime=salobj.current_tai(), **max_velocity_kwargs
                    )
                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_trackTarget.start(timeout=1)
                    await self.assert_next_sample(
                        self.remote.evt_atMountState,
                        state=AtMountState.TRACKINGDISABLED,
                    )
                    await self.fault_to_enabled()

            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
            )

            # a target that is (way) out of bounds at the specified time;
            # failure puts the CSC into the FAULT state
            way_out_kwargs = good_target_kwargs.copy()
            way_out_kwargs["elevation"] = 85
            way_out_kwargs["elevationVelocity"] = -2
            self.remote.cmd_trackTarget.set(
                taiTime=salobj.current_tai() + 10, **way_out_kwargs
            )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_trackTarget.start(timeout=1)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )
            await self.fault_to_enabled()

    async def test_brake_and_drive_status_events(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY):

            # Axes start disabled.
            for event in self.brake_events:
                await self.assert_next_sample(event, engaged=True)

            for event in self.drive_status_events:
                await self.assert_next_sample(event, enable=False)

            # Send to DISABLED state; axes remain disabled
            await self.remote.cmd_start.start()

            # Send to ENABLED state; this should enable
            # elevation, azimuth and NA2 axes.
            await self.remote.cmd_enable.start()

            for event in self.brake_events:
                if event is self.remote.evt_nasmyth2Brake:
                    continue
                await self.assert_next_sample(event, engaged=False)

            for event in self.drive_status_events:
                if event in (
                    self.remote.evt_nasmyth2DriveStatus,
                    self.remote.evt_m3DriveStatus,
                ):
                    continue
                await self.assert_next_sample(event, enable=True)

            # Send to DISABLED state; all axes should be disabled.
            await self.remote.cmd_disable.start()

            for event in self.brake_events:
                if event is self.remote.evt_nasmyth2Brake:
                    continue
                await self.assert_next_sample(event, engaged=True)

            for event in self.drive_status_events:
                if event in (
                    self.remote.evt_nasmyth2DriveStatus,
                    self.remote.evt_m3DriveStatus,
                ):
                    continue
                await self.assert_next_sample(event, enable=False)

    async def test_standard_state_transitions(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY):
            await self.check_standard_state_transitions(
                enabled_commands=(
                    "startTracking",
                    "trackTarget",
                    "setInstrumentPort",
                    "stopTracking",
                )
            )

    async def test_set_instrument_port(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY):
            # Change states manually to make the test compatible
            # with both ts_salobj 6.0 and 6.1: 6.0 does not output
            # evt_nasmyth1DriveStatus with enable=False
            # if initial_state=salobj.State.ENABLE).
            # Once we are not longer using salobj 6, it is safe to
            # remove the following line and specify
            # ``initial_state=salobj.State.ENABLED`` above
            await salobj.set_summary_state(self.remote, state=salobj.State.ENABLED)
            self.csc.configure(
                max_velocity=(100,) * 5,
                max_acceleration=(200,) * 5,
            )

            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.NASMYTH1
            )
            await self.assert_next_sample(
                self.remote.evt_nasmyth1DriveStatus, enable=False
            )
            await self.assert_next_sample(
                self.remote.evt_nasmyth1DriveStatus, enable=True
            )
            await self.assert_next_sample(
                self.remote.evt_nasmyth2DriveStatus, enable=False
            )

            await self.remote.cmd_setInstrumentPort.set_start(
                port=M3ExitPort.PORT3, timeout=STD_TIMEOUT
            )
            await self.assert_next_sample(
                self.remote.evt_m3PortSelected, selected=M3ExitPort.PORT3
            )
            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.INMOTION
            )

            # Nasmyth1 should now be disabled
            # and Nasmyth1 should remain disabled.
            await self.assert_next_sample(
                self.remote.evt_nasmyth1DriveStatus, enable=False
            )
            data = self.remote.evt_nasmyth2DriveStatus.get()
            self.assertFalse(data.enable)

            start_tai = salobj.current_tai()

            await asyncio.sleep(0.2)

            # Attempts to start tracking should fail while M3 is moving.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_startTracking.start()

            actuator = self.csc.actuators[ATMCSSimulator.Axis.M3]
            curr_segment = actuator.path.at(salobj.current_tai())
            self.assertNotEqual(curr_segment.velocity, 0)

            # M3 is pointing to Port 3; neither rotator should be enabled.
            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.PORT3, timeout=5
            )
            dt = salobj.current_tai() - start_tai
            print(f"test_set_instrument_port M3 rotation took {dt:0.2f} sec")
            data = self.remote.evt_nasmyth1DriveStatus.get()
            self.assertFalse(data.enable)
            data = self.remote.evt_nasmyth2DriveStatus.get()
            self.assertFalse(data.enable)

            await self.remote.cmd_setInstrumentPort.set_start(
                port=M3ExitPort.NASMYTH2, timeout=STD_TIMEOUT
            )

            start_tai = salobj.current_tai()
            await self.assert_next_sample(
                self.remote.evt_m3PortSelected, selected=M3ExitPort.NASMYTH2
            )
            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.INMOTION
            )

            # Both rotators should remain disabled.
            data = self.remote.evt_nasmyth1DriveStatus.get()
            self.assertFalse(data.enable)
            data = self.remote.evt_nasmyth2DriveStatus.get()
            self.assertFalse(data.enable)
            self.remote.evt_nasmyth2DriveStatus.flush()

            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.NASMYTH2, timeout=5
            )
            dt = salobj.current_tai() - start_tai
            print(f"test_set_instrument_port M3 rotation took {dt:0.2f} sec")

            # M3 is pointing to Nasmyth2; that rotator
            # should be enabled and Nasmyth1 should not.
            await self.assert_next_sample(
                self.remote.evt_nasmyth2DriveStatus, enable=True
            )
            data = self.remote.evt_nasmyth1DriveStatus.get()
            self.assertFalse(data.enable)

    async def test_bin_script(self):
        await self.check_bin_script(
            name="ATMCS", index=None, exe_name="run_atmcs_simulator.py"
        )

    async def test_track(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            self.csc.configure(
                max_velocity=(100,) * 5,
                max_acceleration=(200,) * 5,
            )

            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.NASMYTH1
            )

            # M3 should be in position, the other axes should not.
            for event in self.in_position_events:
                desired_in_position = event is self.remote.evt_m3InPosition
                await self.assert_next_sample(event, inPosition=desired_in_position)
            await self.assert_next_sample(
                self.remote.evt_allAxesInPosition, inPosition=False
            )

            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)

            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
            )

            # attempts to set instrument port should fail
            with salobj.assertRaisesAckError():
                self.remote.cmd_setInstrumentPort.set(port=1)
                await self.remote.cmd_setInstrumentPort.start()

            start_tai = salobj.current_tai()
            path_dict = dict(
                elevation=simactuators.path.PathSegment(
                    tai=start_tai, position=75, velocity=0.001
                ),
                azimuth=simactuators.path.PathSegment(
                    tai=start_tai, position=5, velocity=-0.001
                ),
                nasmyth1RotatorAngle=simactuators.path.PathSegment(
                    tai=start_tai, position=1, velocity=-0.001
                ),
            )
            trackId = 20  # arbitary
            while True:
                tai = salobj.current_tai() + 0.1  # offset is arbitrary but reasonable
                target_kwargs = self.compute_track_target_kwargs(
                    tai=tai, path_dict=path_dict, trackId=trackId
                )
                await self.remote.cmd_trackTarget.set_start(**target_kwargs, timeout=1)

                target = await self.remote.evt_target.next(flush=False, timeout=1)
                self.assertTargetsAlmostEqual(self.remote.cmd_trackTarget.data, target)

                data = self.remote.evt_allAxesInPosition.get()
                if data.inPosition:
                    break

                if salobj.current_tai() - start_tai > 5:
                    raise self.fail("Timed out waiting for slew to finish")

                await asyncio.sleep(0.5)

            print(f"test_track slew took {salobj.current_tai() - start_tai:0.2f} sec")

            with self.assertRaises(asyncio.TimeoutError):
                await self.remote.evt_target.next(flush=False, timeout=0.1)

            for event in self.in_position_events:
                if event is self.remote.evt_m3InPosition:
                    continue  # M3 was already in position.
                if event is self.remote.evt_nasmyth2RotatorInPosition:
                    continue  # Nasmyth2 is not in use.
                await self.assert_next_sample(event, inPosition=True)

            await self.remote.cmd_stopTracking.start(timeout=1)

    async def test_late_track_target(self):
        # Use a short tracking interval so the test runs quickly.
        max_tracking_interval = 0.2
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            # Get the initial summary state, so `fault_to_enabled` sees FAULT.
            await self.assert_next_summary_state(salobj.State.ENABLED)

            self.csc.configure(
                max_tracking_interval=max_tracking_interval,
                max_velocity=(100,) * 5,
                max_acceleration=(200,) * 5,
            )
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.NASMYTH1
            )

            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
            )

            # wait too long for trackTarget
            await self.assert_next_sample(
                self.remote.evt_atMountState,
                state=AtMountState.TRACKINGDISABLED,
                timeout=max_tracking_interval + 0.2,
            )
            await self.fault_to_enabled()

            # try again, and this time send a trackTarget command
            # before waiting too long
            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
            )

            await self.remote.cmd_trackTarget.set_start(
                elevation=10, taiTime=salobj.current_tai(), trackId=20, timeout=1
            )

            # wait too long for trackTarget
            await self.assert_next_sample(
                self.remote.evt_atMountState,
                state=AtMountState.STOPPING,
                timeout=max_tracking_interval + 0.2,
            )

            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

    async def test_stop_tracking_while_slewing(self):
        """Call stopTracking while tracking, before a slew is done."""
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

            await self.assert_next_sample(
                self.remote.evt_m3State, state=M3State.NASMYTH1
            )

            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
            )

            start_tai = salobj.current_tai()
            path_dict = dict(
                elevation=simactuators.path.PathSegment(tai=start_tai, position=45),
                azimuth=simactuators.path.PathSegment(tai=start_tai, position=100),
                nasmyth1RotatorAngle=simactuators.path.PathSegment(
                    tai=start_tai, position=90
                ),
            )
            trackId = 35  # arbitary

            tai = start_tai + 0.1  # offset is arbitrary but reasonable
            target_kwargs = self.compute_track_target_kwargs(
                tai=tai, path_dict=path_dict, trackId=trackId
            )
            await self.remote.cmd_trackTarget.set_start(**target_kwargs, timeout=1)

            await asyncio.sleep(0.2)

            await self.remote.cmd_stopTracking.start(timeout=1)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.STOPPING
            )

            await asyncio.sleep(0.2)  # Give events time to arrive.

            for event in self.in_position_events:
                desired_in_position = event is self.remote.evt_m3InPosition
                await self.assert_next_sample(event, inPosition=desired_in_position)

            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

            for actuator in self.csc.actuators:
                self.assertEqual(actuator.kind(), actuator.Kind.Stopped)

    async def test_disable_while_slewing(self):
        """Call disable while tracking, before a slew is done."""
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            state = await self.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGENABLED
            )

            start_tai = salobj.current_tai()
            path_dict = dict(
                elevation=simactuators.path.PathSegment(tai=start_tai, position=45),
                azimuth=simactuators.path.PathSegment(tai=start_tai, position=100),
                nasmyth1RotatorAngle=simactuators.path.PathSegment(
                    tai=start_tai, position=90
                ),
            )
            trackId = 35  # arbitary

            tai = start_tai + 0.1  # offset is arbitrary but reasonable
            target_kwargs = self.compute_track_target_kwargs(
                tai=tai, path_dict=path_dict, trackId=trackId
            )
            await self.remote.cmd_trackTarget.set_start(**target_kwargs, timeout=1)

            await asyncio.sleep(0.2)

            await self.remote.cmd_disable.start(timeout=1)
            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.STOPPING
            )

            await asyncio.sleep(0.2)  # Give events time to arrive.

            for event in self.in_position_events:
                desired_in_position = event is self.remote.evt_m3InPosition
                await self.assert_next_sample(event, inPosition=desired_in_position)

            await self.assert_next_sample(
                self.remote.evt_atMountState, state=AtMountState.TRACKINGDISABLED
            )

    def assertTargetsAlmostEqual(self, target1, target2):
        """Assert two targets are approximately equal.

        Parameters
        ----------
        target1, target2 : `any`
            The targets to compare. These may be instances of trackTarget
            command data or target event data.
        """
        for field in (
            "azimuth",
            "azimuthVelocity",
            "elevation",
            "elevationVelocity",
            "nasmyth1RotatorAngle",
            "nasmyth1RotatorAngleVelocity",
            "nasmyth2RotatorAngle",
            "nasmyth2RotatorAngleVelocity",
            "taiTime",
            "trackId",
        ):
            self.assertAlmostEqual(getattr(target1, field), getattr(target2, field))

    def compute_track_target_kwargs(self, tai, path_dict, trackId):
        """Compute keyword arguments for the trackTarget command.

        Parameters
        ----------
        tai : `float`
            TAI date, unix seconds.
        path_dict : `dict`
            Dict of axis name: path (an lsst.ts.simactuators.path.Path
            or PathSegment).
        trackId : `int`
            Tracking ID.
        """
        target_kwargs = dict(taiTime=tai, trackId=trackId)
        for axis_name, path in path_dict.items():
            segment = path.at(tai)
            target_kwargs[axis_name] = segment.position
            target_kwargs[f"{axis_name}Velocity"] = segment.velocity
        return target_kwargs


if __name__ == "__main__":
    unittest.main()
