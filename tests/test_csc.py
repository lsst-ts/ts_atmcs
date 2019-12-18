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

from lsst.ts import salobj
from lsst.ts import simactuators
from lsst.ts import ATMCSSimulator
from lsst.ts.idl.enums.ATMCS import AtMountState, M3ExitPort, M3State

STD_TIMEOUT = 5  # standard timeout, seconds


class Harness:
    def __init__(self, initial_state):
        salobj.test_utils.set_random_lsst_dds_domain()
        self.csc = ATMCSSimulator.ATMCSCsc(initial_state=initial_state)
        self.remote = salobj.Remote(domain=self.csc.domain, name="ATMCS", index=0)

    async def next_event(self, name, flush=False, timeout=1):
        """Wait for and return the next sample of a specified event topic.
        """
        try:
            event = getattr(self.remote, f"evt_{name}")
            return await event.next(flush=flush, timeout=timeout)
        except Exception as e:
            raise RuntimeError(f"Could not get data for event {name}") from e

    def get_event(self, name):
        """Get the most recent sample of a specified event topic.
        """
        try:
            event = getattr(self.remote, f"evt_{name}")
            return event.get()
        except Exception as e:
            raise RuntimeError(f"Could not get data for event {name}") from e

    async def __aenter__(self):
        await self.csc.start_task
        return self

    async def __aexit__(self, *args):
        await self.csc.close()


class CscTestCase(unittest.TestCase):
    def setUp(self):
        self.axis_names = (  # names of axes for trackTarget command
            "elevation",
            "azimuth",
            "nasmyth1RotatorAngle",
            "nasmyth2RotatorAngle",
        )
        self.enable_names = (
            "elevationDriveStatus",
            "azimuthDrive1Status",
            "azimuthDrive2Status",
            "nasmyth1DriveStatus",
            "nasmyth2DriveStatus",
            "m3DriveStatus",
        )

        self.brake_names = (
            "azimuthBrake1",
            "azimuthBrake2",
            "elevationBrake",
            "nasmyth1Brake",
            "nasmyth2Brake",
        )

        self.in_position_names = (
            "elevationInPosition",
            "azimuthInPosition",
            "nasmyth1RotatorInPosition",
            "nasmyth2RotatorInPosition",
            "m3InPosition",
        )

    async def fault_to_enabled(self, harness):
        """Check that the CSC is in FAULT state and enable it.

        Assumes that the FAULT state has not yet been read from the remote.
        """
        state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
        self.assertEqual(state.summaryState, salobj.State.FAULT)

        await harness.remote.cmd_standby.start(timeout=STD_TIMEOUT)
        state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
        self.assertEqual(state.summaryState, salobj.State.STANDBY)

        await harness.remote.cmd_start.start(timeout=STD_TIMEOUT)
        state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
        self.assertEqual(state.summaryState, salobj.State.DISABLED)

        await harness.remote.cmd_enable.start(timeout=STD_TIMEOUT)
        state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
        self.assertEqual(state.summaryState, salobj.State.ENABLED)

    def test_initial_info(self):
        """Check that all events and telemetry are output at startup

        except the m3PortSelected event
        """
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                for event_name in harness.csc.salinfo.event_names:
                    if event_name in (
                        "m3PortSelected",  # output by setInstrumentPort
                        "target",  # output by trackTarget
                        "summaryState",  # already read
                        "softwareVersions",  # not yet supported by salobj
                        "appliedSettingsMatchStart", "detailedState",
                        "errorCode", "logMessage", "settingVersions",
                    ):
                        continue
                    with self.subTest(event_name=event_name):
                        await harness.next_event(event_name)

                timeout = STD_TIMEOUT
                for tel_name in harness.csc.salinfo.telemetry_names:
                    with self.subTest(tel_name=tel_name):
                        tel = getattr(harness.remote, f"tel_{tel_name}")
                        await tel.next(flush=False, timeout=timeout)
                    timeout = 0.1

        asyncio.get_event_loop().run_until_complete(doit())

    def test_invalid_track_target(self):
        """Test all reasons trackTarget may be rejected.
        """
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                min_commanded_position = (5, -270, -165, -165, 0)
                max_commanded_position = (90, 270, 165, 165, 180)
                max_velocity = (100,)*5
                harness.csc.configure(
                    min_commanded_position=min_commanded_position,
                    max_commanded_position=max_commanded_position,
                    max_velocity=max_velocity,
                    max_acceleration=(200,)*5,
                )
                good_target_kwargs = dict((name, 0) for name in self.axis_names)
                # elevation does not have 0 in its valid range
                good_target_kwargs[self.axis_names[0]] = min_commanded_position[0]
                # zero velocity as well
                velocity_kwargs = dict((f"{name}Velocity", 0) for name in self.axis_names)
                good_target_kwargs.update(velocity_kwargs)
                good_target_kwargs["trackId"] = 137

                state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

                # cannot send trackTarget while tracking disabled;
                # this error does not change the summary state
                harness.remote.cmd_trackTarget.set(time=salobj.current_tai(), **good_target_kwargs)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_trackTarget.start(timeout=1)

                # the rejected target should not be output as an event
                with self.assertRaises(asyncio.TimeoutError):
                    await harness.remote.evt_target.next(flush=False, timeout=0.1)

                # enable tracking and try again; this time it should work
                await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGENABLED)
                await harness.remote.cmd_trackTarget.start(timeout=1)
                await asyncio.sleep(0.1)

                # disable tracking and re-enable, so state is TrackingEnabled
                await harness.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.STOPPING)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

                for axis in ATMCSSimulator.Axis:
                    if axis == ATMCSSimulator.Axis.M3:
                        continue  # trackTarget doesn't accept M3
                    with self.subTest(axis=axis):
                        await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                        data = await harness.next_event("atMountState")
                        self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                        min_position_kwargs = good_target_kwargs.copy()
                        min_position_kwargs[self.axis_names[axis]] = min_commanded_position[axis] - 0.000001
                        harness.remote.cmd_trackTarget.set(time=salobj.current_tai(), **min_position_kwargs)
                        with salobj.assertRaisesAckError():
                            await harness.remote.cmd_trackTarget.start(timeout=1)
                        data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                        self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)
                        await self.fault_to_enabled(harness)

                        await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                        data = await harness.next_event("atMountState")
                        self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                        max_position_kwargs = good_target_kwargs.copy()
                        max_position_kwargs[self.axis_names[axis]] = max_commanded_position[axis] + 0.000001
                        harness.remote.cmd_trackTarget.set(time=salobj.current_tai(), **max_position_kwargs)
                        with salobj.assertRaisesAckError():
                            await harness.remote.cmd_trackTarget.start(timeout=1)
                        data = await harness.next_event("atMountState")
                        self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)
                        await self.fault_to_enabled(harness)

                        await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                        data = await harness.next_event("atMountState")
                        self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                        max_velocity_kwargs = good_target_kwargs.copy()
                        max_velocity_kwargs[f"{self.axis_names[axis]}Velocity"] = \
                            max_velocity[axis] + 0.000001
                        harness.remote.cmd_trackTarget.set(time=salobj.current_tai(), **max_velocity_kwargs)
                        with salobj.assertRaisesAckError():
                            await harness.remote.cmd_trackTarget.start(timeout=1)
                        data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                        self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)
                        await self.fault_to_enabled(harness)

                await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                # a target that is (way) out of bounds at the specified time;
                # failure puts the CSC into the FAULT state
                way_out_kwargs = good_target_kwargs.copy()
                way_out_kwargs["elevation"] = 85
                way_out_kwargs["elevationVelocity"] = -2
                harness.remote.cmd_trackTarget.set(time=salobj.current_tai() + 10, **way_out_kwargs)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_trackTarget.start(timeout=1)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)
                await self.fault_to_enabled(harness)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_standard_state_transitions(self):
        """Test standard CSC state transitions.
        """
        async def doit():
            async with Harness(initial_state=salobj.State.STANDBY) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.STANDBY)

                print("Starting test; check that brakes are engaged")
                for event_name in self.brake_names:
                    data = await harness.next_event(event_name)
                    self.assertTrue(data.engaged)

                for event_name in self.enable_names:
                    data = await harness.next_event(event_name)
                    self.assertFalse(data.enable)

                # send start; new state is DISABLED
                print("Send start; brakes are still engaged")
                await harness.remote.cmd_start.start()
                self.assertEqual(harness.csc.summary_state, salobj.State.DISABLED)
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.DISABLED)

                # send enable; new state is ENABLED
                print("Send enable; check that brakes are disengaged")
                await harness.remote.cmd_enable.start()
                self.assertEqual(harness.csc.summary_state, salobj.State.ENABLED)
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                # the following axes should be enabled:
                # elevation, azimuth and NA2
                for event_name in self.brake_names:
                    if event_name == "nasmyth2Brake":
                        continue
                    data = await harness.next_event(event_name)
                    self.assertFalse(data.engaged)

                for event_name in self.enable_names:
                    if event_name in ("nasmyth2DriveStatus", "m3DriveStatus"):
                        continue
                    data = await harness.next_event(event_name)
                    self.assertTrue(data.enable)

                # send disable; new state is DISABLED
                await harness.remote.cmd_disable.start()
                self.assertEqual(harness.csc.summary_state, salobj.State.DISABLED)
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.DISABLED)

                for event_name in self.brake_names:
                    if event_name == "nasmyth2Brake":
                        continue
                    data = await harness.next_event(event_name)
                    self.assertTrue(data.engaged)

                for event_name in self.enable_names:
                    if event_name in ("nasmyth2DriveStatus", "m3DriveStatus"):
                        continue
                    data = await harness.next_event(event_name)
                    self.assertFalse(data.enable)

                # send standby; new state is STANDBY
                await harness.remote.cmd_standby.start()
                self.assertEqual(harness.csc.summary_state, salobj.State.STANDBY)
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.STANDBY)

                # send exitControl; new state is OFFLINE
                await harness.remote.cmd_exitControl.start()
                self.assertEqual(harness.csc.summary_state, salobj.State.OFFLINE)
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.OFFLINE)

                await asyncio.wait_for(harness.csc.done_task, 5)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_set_instrument_port(self):
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                harness.csc.configure(
                    max_velocity=(100,)*5,
                    max_acceleration=(200,)*5,
                )
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                data = await harness.next_event("m3State")
                self.assertEqual(data.state, M3State.NASMYTH1)

                await harness.remote.cmd_setInstrumentPort.set_start(port=M3ExitPort.PORT3,
                                                                     timeout=STD_TIMEOUT)

                data = await harness.next_event("m3PortSelected")
                self.assertEqual(data.selected, M3ExitPort.PORT3)
                data = await harness.next_event("m3State")
                self.assertEqual(data.state, M3State.INMOTION)

                # neither rotator should be enabled
                data = harness.remote.evt_nasmyth1DriveStatus.get(flush=True)
                self.assertFalse(data.enable)
                data = harness.remote.evt_nasmyth2DriveStatus.get(flush=True)
                self.assertFalse(data.enable)

                start_tai = salobj.current_tai()

                await asyncio.sleep(0.2)

                # attempts to start tracking should fail while M3 is moving
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_startTracking.start()

                actuator = harness.csc.actuators[ATMCSSimulator.Axis.M3]
                curr_segment = actuator.path.at(salobj.current_tai())
                self.assertNotEqual(curr_segment.velocity, 0)

                # M3 is pointing to Port 3; neither rotator should be enabled
                data = await harness.next_event("m3State", timeout=5)
                dt = salobj.current_tai() - start_tai
                print(f"test_set_instrument_port M3 rotation took {dt:0.2f} sec")
                self.assertEqual(data.state, M3State.PORT3)
                data = harness.remote.evt_nasmyth1DriveStatus.get(flush=True)
                self.assertFalse(data.enable)
                data = harness.remote.evt_nasmyth2DriveStatus.get(flush=True)
                self.assertFalse(data.enable)

                await harness.remote.cmd_setInstrumentPort.set_start(port=M3ExitPort.NASMYTH2,
                                                                     timeout=STD_TIMEOUT)

                start_tai = salobj.current_tai()
                data = await harness.next_event("m3State", timeout=2)
                self.assertEqual(data.state, M3State.INMOTION)

                # neither rotator should be enabled
                data = harness.remote.evt_nasmyth1DriveStatus.get(flush=True)
                self.assertFalse(data.enable)
                data = harness.remote.evt_nasmyth2DriveStatus.get(flush=True)
                self.assertFalse(data.enable)

                data = await harness.next_event("m3State", timeout=5)
                dt = salobj.current_tai() - start_tai
                print(f"test_set_instrument_port M3 rotation took {dt:0.2f} sec")
                self.assertEqual(data.state, M3State.NASMYTH2)

                # M3 is pointing to Nasmyth2; that rotator
                # should be enabled and Nasmyth1 should not
                data = await harness.next_event("nasmyth2DriveStatus")
                self.assertTrue(data.enable)
                data = harness.remote.evt_nasmyth1DriveStatus.get()
                self.assertFalse(data.enable)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_track(self):
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                harness.csc.configure(
                    max_velocity=(100,)*5,
                    max_acceleration=(200,)*5,
                )
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

                data = await harness.next_event("m3State")
                self.assertEqual(data.state, M3State.NASMYTH1)

                for event_name in self.in_position_names:
                    data = await harness.next_event(event_name)
                    if event_name.startswith("m3"):
                        self.assertTrue(data.inPosition)
                    else:
                        self.assertFalse(data.inPosition)

                data = await harness.next_event("allAxesInPosition")
                self.assertFalse(data.inPosition)

                await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)

                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                # attempts to set instrument port should fail
                with salobj.assertRaisesAckError():
                    harness.remote.cmd_setInstrumentPort.set(port=1)
                    await harness.remote.cmd_setInstrumentPort.start()

                start_tai = salobj.current_tai()
                paths = dict(
                    elevation=simactuators.path.PathSegment(tai=start_tai, position=6, velocity=0.001),
                    azimuth=simactuators.path.PathSegment(tai=start_tai, position=5, velocity=-0.001),
                    nasmyth1RotatorAngle=simactuators.path.PathSegment(tai=start_tai,
                                                                       position=1,
                                                                       velocity=-0.001),
                )
                trackId = 20  # arbitary
                while True:
                    tai = salobj.current_tai() + 0.1  # offset is arbitrary but reasonable
                    harness.remote.cmd_trackTarget.set(time=tai, trackId=trackId)
                    for axis_name, path in paths.items():
                        segment = path.at(tai)
                        kwargs = {
                            axis_name: segment.position,
                            f"{axis_name}Velocity": segment.velocity,
                        }
                        harness.remote.cmd_trackTarget.set(**kwargs)
                    await harness.remote.cmd_trackTarget.start(timeout=1)

                    target = await harness.remote.evt_target.next(flush=False, timeout=1)
                    self.assertTargetsAlmostEqual(harness.remote.cmd_trackTarget.data, target)

                    data = harness.remote.evt_allAxesInPosition.get()
                    if data.inPosition:
                        break

                    if salobj.current_tai() - start_tai > 5:
                        raise self.fail("Timed out waiting for slew to finish")

                    await asyncio.sleep(0.5)

                print(f"test_track slew took {salobj.current_tai() - start_tai:0.2f} sec")

                with self.assertRaises(asyncio.TimeoutError):
                    await harness.remote.evt_target.next(flush=False, timeout=0.1)

                for event_name in self.in_position_names:
                    if event_name.startswith("m3"):
                        continue  # already was in position
                    if event_name.startswith("nasmyth2"):
                        continue  # axis not in use
                    data = await harness.next_event(event_name)
                    self.assertTrue(data.inPosition)

                await harness.remote.cmd_stopTracking.start(timeout=1)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_late_track_target(self):
        async def doit():
            # short so test runs quickly
            max_tracking_interval = 0.2
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                harness.csc.configure(
                    max_tracking_interval=max_tracking_interval,
                    max_velocity=(100,)*5,
                    max_acceleration=(200,)*5,
                )
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

                data = await harness.next_event("m3State")
                self.assertEqual(data.state, M3State.NASMYTH1)

                await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                # wait too long for trackTarget
                data = await harness.next_event("atMountState", timeout=max_tracking_interval + 0.2)
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)
                await self.fault_to_enabled(harness)

                # try again, and this time send a trackTarget command
                # before waiting too long
                await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                harness.remote.cmd_trackTarget.set(
                    elevation=10,
                    time=salobj.current_tai(),
                    trackId=20,  # arbitary
                )
                await harness.remote.cmd_trackTarget.start(timeout=1)

                # wait too long for trackTarget
                data = await harness.next_event("atMountState", timeout=max_tracking_interval + 0.2)
                self.assertEqual(data.state, AtMountState.STOPPING)

                data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_stop_tracking_while_slewing(self):
        """Call stopTracking while tracking, before a slew is done.
        """
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

                data = await harness.next_event("m3State")
                self.assertEqual(data.state, M3State.NASMYTH1)

                await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                start_tai = salobj.current_tai()
                paths = dict(
                    elevation=simactuators.path.PathSegment(tai=start_tai, position=45),
                    azimuth=simactuators.path.PathSegment(tai=start_tai, position=100),
                    nasmyth1RotatorAngle=simactuators.path.PathSegment(tai=start_tai, position=90),
                )
                trackId = 35  # arbitary

                tai = start_tai + 0.1  # offset is arbitrary but reasonable
                harness.remote.cmd_trackTarget.set(time=tai, trackId=trackId)
                for axis_name, path in paths.items():
                    segment = path.at(tai)
                    kwargs = {
                        axis_name: segment.position,
                        f"{axis_name}Velocity": segment.velocity,
                    }
                    harness.remote.cmd_trackTarget.set(**kwargs)
                await harness.remote.cmd_trackTarget.start(timeout=1)

                await asyncio.sleep(0.2)

                await harness.remote.cmd_stopTracking.start(timeout=1)
                data = await harness.next_event("atMountState")
                self.assertEqual(data.state, AtMountState.STOPPING)

                await asyncio.sleep(0.2)  # give events time to arrive

                for event_name in self.in_position_names:
                    data = harness.get_event(event_name)
                    if event_name.startswith("m3"):
                        self.assertTrue(data.inPosition)
                    else:
                        self.assertFalse(data.inPosition)

                data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

                for actuator in harness.csc.actuators:
                    self.assertEqual(actuator.kind(), actuator.Kind.Stopped)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_disable_while_slewing(self):
        """Call disable while tracking, before a slew is done.
        """
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)
                data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

                await harness.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
                data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                self.assertEqual(data.state, AtMountState.TRACKINGENABLED)

                start_tai = salobj.current_tai()
                paths = dict(
                    elevation=simactuators.path.PathSegment(tai=start_tai, position=45),
                    azimuth=simactuators.path.PathSegment(tai=start_tai, position=100),
                    nasmyth1RotatorAngle=simactuators.path.PathSegment(tai=start_tai, position=90),
                )
                trackId = 35  # arbitary

                tai = start_tai + 0.1  # offset is arbitrary but reasonable
                harness.remote.cmd_trackTarget.set(time=tai, trackId=trackId)
                for axis_name, path in paths.items():
                    segment = path.at(tai)
                    kwargs = {
                        axis_name: segment.position,
                        f"{axis_name}Velocity": segment.velocity,
                    }
                    harness.remote.cmd_trackTarget.set(**kwargs)
                await harness.remote.cmd_trackTarget.start(timeout=1)

                await asyncio.sleep(0.2)

                await harness.remote.cmd_disable.start(timeout=1)
                data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                self.assertEqual(data.state, AtMountState.STOPPING)

                await asyncio.sleep(0.2)  # give events time to arrive

                for event_name in self.in_position_names:
                    data = harness.get_event(event_name)
                    if event_name.startswith("m3"):
                        self.assertTrue(data.inPosition)
                    else:
                        self.assertFalse(data.inPosition)

                data = await harness.next_event("atMountState", timeout=STD_TIMEOUT)
                self.assertEqual(data.state, AtMountState.TRACKINGDISABLED)

        asyncio.get_event_loop().run_until_complete(doit())

    def assertTargetsAlmostEqual(self, target1, target2):
        """Assert two targets are approximately equal.

        Parameters
        ----------
        target1, target2 : `any`
            The targets to compare. These may be instances of trackTarget
            command data or target event data.
        """
        for field in ("azimuth", "azimuthVelocity",
                      "elevation", "elevationVelocity",
                      "nasmyth1RotatorAngle", "nasmyth1RotatorAngleVelocity",
                      "nasmyth2RotatorAngle", "nasmyth2RotatorAngleVelocity",
                      "time", "trackId"):
            self.assertAlmostEqual(getattr(target1, field), getattr(target2, field))


if __name__ == "__main__":
    unittest.main()
