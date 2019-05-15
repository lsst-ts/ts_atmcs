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
from lsst.ts import ATMCSSimulator

import SALPY_ATMCS


class Harness:
    def __init__(self, initial_state):
        salobj.test_utils.set_random_lsst_dds_domain()
        self.remote = salobj.Remote(SALPY_ATMCS, index=0)
        self.csc = ATMCSSimulator.ATMCSCsc(
            initial_state=initial_state, initial_simulation_mode=1)

    async def next_evt(self, name, flush=False, timeout=1):
        try:
            evt = getattr(self.remote, f"evt_{name}")
            return await evt.next(flush=flush, timeout=timeout)
        except Exception as e:
            raise RuntimeError(f"Could not get data for event {name}") from e

    def get_evt(self, name):
        try:
            evt = getattr(self.remote, f"evt_{name}")
            return evt.get()
        except Exception as e:
            raise RuntimeError(f"Could not get data for event {name}") from e


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

        await harness.remote.cmd_standby.start(timeout=2)
        state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
        self.assertEqual(state.summaryState, salobj.State.STANDBY)

        await harness.remote.cmd_start.start(timeout=2)
        state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
        self.assertEqual(state.summaryState, salobj.State.DISABLED)

        await harness.remote.cmd_enable.start(timeout=2)
        state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
        self.assertEqual(state.summaryState, salobj.State.ENABLED)

    def test_initial_info(self):
        """Check that all events and telemetry are output at startup

        except the m3PortSelected event
        """

        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            for evt_name in harness.csc.salinfo.manager.getEventNames():
                if evt_name in (
                    "m3PortSelected",  # output by setInstrumentPort
                    "target",  # output by trackTarget
                    "summaryState",  # already read
                    "appliedSettingsMatchStart", "detailedState",
                    "errorCode", "logMessage", "settingVersions",
                ):
                    continue
                await harness.next_evt(evt_name)

            for tel_name in harness.csc.salinfo.manager.getTelemetryNames():
                tel = getattr(harness.remote, f"tel_{tel_name}")
                await tel.next(flush=False, timeout=0.1)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_invalid_track_target(self):
        """Test all reasons trackTarget may be rejected."""
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            pmin_cmd = (5, -270, -165, -165, 0)
            pmax_cmd = (90, 270, 165, 165, 180)
            vmax = (100,)*5
            harness.csc.configure(
                pmin_cmd=pmin_cmd,
                pmax_cmd=pmax_cmd,
                vmax=vmax,
                amax=(200,)*5,
            )
            good_target_kwargs = dict((name, 0) for name in self.axis_names)
            # elevation does not have 0 in its valid range
            good_target_kwargs[self.axis_names[0]] = pmin_cmd[0]
            # zero velocity as well
            vel_kwargs = dict((f"{name}Velocity", 0) for name in self.axis_names)
            good_target_kwargs.update(vel_kwargs)
            good_target_kwargs["trackId"] = 137

            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            # cannot send trackTarget while tracking disabled;
            # this error does not change the summary state
            harness.remote.cmd_trackTarget.set(time=ATMCSSimulator.curr_tai(), **good_target_kwargs)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)

            # the rejected target should not be output as an event
            with self.assertRaises(asyncio.TimeoutError):
                await harness.remote.evt_target.next(flush=False, timeout=0.1)

            # enable tracking and try again; this time it should work
            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)
            await harness.remote.cmd_trackTarget.start(timeout=1)
            await asyncio.sleep(0.1)

            # disable tracking and re-enable, so state is TrackingEnabled
            await harness.remote.cmd_stopTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_Stopping)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            for axis in ATMCSSimulator.Axis:
                if axis == ATMCSSimulator.Axis.M3:
                    continue  # trackTarget doesn't accept M3
                with self.subTest(axis=axis):
                    await harness.remote.cmd_startTracking.start(timeout=2)
                    data = await harness.next_evt("atMountState")
                    self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

                    pmin_kwargs = good_target_kwargs.copy()
                    pmin_kwargs[self.axis_names[axis]] = pmin_cmd[axis] - 0.000001
                    harness.remote.cmd_trackTarget.set(time=ATMCSSimulator.curr_tai(), **pmin_kwargs)
                    with salobj.assertRaisesAckError():
                        await harness.remote.cmd_trackTarget.start(timeout=1)
                    data = await harness.next_evt("atMountState", timeout=2)
                    self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)
                    await self.fault_to_enabled(harness)

                    await harness.remote.cmd_startTracking.start(timeout=2)
                    data = await harness.next_evt("atMountState")
                    self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

                    pmax_kwargs = good_target_kwargs.copy()
                    pmax_kwargs[self.axis_names[axis]] = pmax_cmd[axis] + 0.000001
                    harness.remote.cmd_trackTarget.set(time=ATMCSSimulator.curr_tai(), **pmax_kwargs)
                    with salobj.assertRaisesAckError():
                        await harness.remote.cmd_trackTarget.start(timeout=1)
                    data = await harness.next_evt("atMountState")
                    self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)
                    await self.fault_to_enabled(harness)

                    await harness.remote.cmd_startTracking.start(timeout=2)
                    data = await harness.next_evt("atMountState")
                    self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

                    vmax_kwargs = good_target_kwargs.copy()
                    vmax_kwargs[f"{self.axis_names[axis]}Velocity"] = vmax[axis] + 0.000001
                    harness.remote.cmd_trackTarget.set(time=ATMCSSimulator.curr_tai(), **vmax_kwargs)
                    with salobj.assertRaisesAckError():
                        await harness.remote.cmd_trackTarget.start(timeout=1)
                    data = await harness.next_evt("atMountState", timeout=2)
                    self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)
                    await self.fault_to_enabled(harness)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            # a target that is (way) out of bounds at the specified time;
            # failure puts the CSC into the FAULT state
            way_out_kwargs = good_target_kwargs.copy()
            way_out_kwargs["elevation"] = 85
            way_out_kwargs["elevationVelocity"] = -2
            harness.remote.cmd_trackTarget.set(time=ATMCSSimulator.curr_tai() + 10, **way_out_kwargs)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)
            await self.fault_to_enabled(harness)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_standard_state_transitions(self):
        """Test standard CSC state transitions.
        """
        async def doit():
            harness = Harness(initial_state=salobj.State.STANDBY)
            self.assertEqual(harness.csc.summary_state, salobj.State.STANDBY)
            # make sure start_task completes
            await asyncio.wait_for(harness.csc.start_task, timeout=2)

            state = await harness.remote.evt_summaryState.next(flush=False, timeout=2)
            self.assertEqual(state.summaryState, salobj.State.STANDBY)

            for evt_name in self.brake_names:
                data = await harness.next_evt(evt_name)
                self.assertTrue(data.engaged)

            for evt_name in self.enable_names:
                data = await harness.next_evt(evt_name)
                self.assertFalse(data.enable)

            # send start; new state is DISABLED
            id_ack = await harness.remote.cmd_start.start()
            self.assertEqual(id_ack.ack.ack, harness.remote.salinfo.lib.SAL__CMD_COMPLETE)
            self.assertEqual(id_ack.ack.error, 0)
            self.assertEqual(harness.csc.summary_state, salobj.State.DISABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=2)
            self.assertEqual(state.summaryState, salobj.State.DISABLED)

            # send enable; new state is ENABLED
            id_ack = await harness.remote.cmd_enable.start()
            self.assertEqual(id_ack.ack.ack, harness.remote.salinfo.lib.SAL__CMD_COMPLETE)
            self.assertEqual(id_ack.ack.error, 0)
            self.assertEqual(harness.csc.summary_state, salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=2)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            for evt_name in self.brake_names:
                data = await harness.next_evt(evt_name)
                self.assertFalse(data.engaged)

            for evt_name in self.enable_names:
                data = await harness.next_evt(evt_name)
                self.assertTrue(data.enable)

            # send disable; new state is DISABLED
            id_ack = await harness.remote.cmd_disable.start()
            self.assertEqual(id_ack.ack.ack, harness.remote.salinfo.lib.SAL__CMD_COMPLETE)
            self.assertEqual(id_ack.ack.error, 0)
            self.assertEqual(harness.csc.summary_state, salobj.State.DISABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=2)
            self.assertEqual(state.summaryState, salobj.State.DISABLED)

            for evt_name in self.brake_names:
                data = await harness.next_evt(evt_name)
                self.assertTrue(data.engaged)

            for evt_name in self.enable_names:
                data = await harness.next_evt(evt_name)
                self.assertFalse(data.enable)

            # send standby; new state is STANDBY
            id_ack = await harness.remote.cmd_standby.start()
            self.assertEqual(id_ack.ack.ack, harness.remote.salinfo.lib.SAL__CMD_COMPLETE)
            self.assertEqual(id_ack.ack.error, 0)
            self.assertEqual(harness.csc.summary_state, salobj.State.STANDBY)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=2)
            self.assertEqual(state.summaryState, salobj.State.STANDBY)

            # send exitControl; new state is OFFLINE
            id_ack = await harness.remote.cmd_exitControl.start()
            self.assertEqual(id_ack.ack.ack, harness.remote.salinfo.lib.SAL__CMD_COMPLETE)
            self.assertEqual(id_ack.ack.error, 0)
            self.assertEqual(harness.csc.summary_state, salobj.State.OFFLINE)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=2)
            self.assertEqual(state.summaryState, salobj.State.OFFLINE)

            await asyncio.wait_for(harness.csc.done_task, 2)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_set_instrument_port(self):
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            harness.csc.configure(
                vmax=(100,)*5,
                amax=(200,)*5,
            )
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            data = await harness.next_evt("m3State")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_M3State_Nasmyth1)

            harness.remote.cmd_setInstrumentPort.set(port=SALPY_ATMCS.ATMCS_shared_M3ExitPort_Port3)
            await harness.remote.cmd_setInstrumentPort.start(timeout=2)

            data = await harness.next_evt("m3PortSelected")
            self.assertEqual(data.selected, SALPY_ATMCS.ATMCS_shared_M3ExitPort_Port3)
            data = await harness.next_evt("m3State")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_M3State_InMotion)

            t0 = ATMCSSimulator.curr_tai()

            await asyncio.sleep(0.2)

            # attempts to start tracking should fail while M3 is moving
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_startTracking.start()

            actuator = harness.csc.actuators[ATMCSSimulator.Axis.M3]
            curr_pos, curr_vel = actuator.curr.pva(ATMCSSimulator.curr_tai())[0:2]
            self.assertNotEqual(curr_vel, 0)

            data = await harness.next_evt("m3State", timeout=5)
            print(f"test_set_instrument_port M3 rotation took {ATMCSSimulator.curr_tai() - t0:0.2f} sec")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_M3State_Port3)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_track(self):
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            harness.csc.configure(
                vmax=(100,)*5,
                amax=(200,)*5,
            )
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            for evt_name in self.in_position_names:
                data = await harness.next_evt(evt_name)
                if evt_name.startswith("m3"):
                    self.assertTrue(data.inPosition)
                else:
                    self.assertFalse(data.inPosition)

            data = await harness.next_evt("allAxesInPosition")
            self.assertFalse(data.inPosition)

            await harness.remote.cmd_startTracking.start(timeout=2)

            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            # attempts to set instrument port should fail
            with salobj.assertRaisesAckError():
                harness.remote.cmd_setInstrumentPort.set(port=1)
                await harness.remote.cmd_setInstrumentPort.start()

            t0 = ATMCSSimulator.curr_tai()
            paths = dict(
                elevation=ATMCSSimulator.path.TPVAJ(t0=t0, p0=6, v0=0.001),
                azimuth=ATMCSSimulator.path.TPVAJ(t0=t0, p0=5, v0=-0.001),
                nasmyth1RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=1, v0=-0.001),
                nasmyth2RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=-2, v0=0.001),
            )
            trackId = 20  # arbitary
            while True:
                t = ATMCSSimulator.curr_tai() + 0.1  # offset is arbitrary but reasonable
                harness.remote.cmd_trackTarget.set(time=t, trackId=trackId)
                for axis_name, path in paths.items():
                    pos, vel = path.pva(t)[0:2]
                    kwargs = {
                        axis_name: pos,
                        f"{axis_name}Velocity": vel,
                    }
                    harness.remote.cmd_trackTarget.set(**kwargs)
                await harness.remote.cmd_trackTarget.start(timeout=1)

                target = await harness.remote.evt_target.next(flush=False, timeout=1)
                self.assertTargetsAlmostEqual(harness.remote.cmd_trackTarget.data, target)

                data = harness.remote.evt_allAxesInPosition.get()
                if data.inPosition:
                    break

                if ATMCSSimulator.curr_tai() - t0 > 5:
                    raise self.fail("Timed out waiting for slew to finish")

                await asyncio.sleep(0.5)

            print(f"test_track slew took {ATMCSSimulator.curr_tai() - t0:0.2f} sec")

            with self.assertRaises(asyncio.TimeoutError):
                await harness.remote.evt_target.next(flush=False, timeout=0.1)

            for evt_name in self.in_position_names:
                if evt_name.startswith("m3"):
                    continue  # already was in position
                data = await harness.next_evt(evt_name)
                self.assertTrue(data.inPosition)

            await harness.remote.cmd_stopTracking.start(timeout=1)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_late_track_target(self):
        async def doit():
            # short so test runs quickly
            max_tracking_interval = 0.2
            harness = Harness(initial_state=salobj.State.ENABLED)
            harness.csc.configure(
                max_tracking_interval=max_tracking_interval,
                vmax=(100,)*5,
                amax=(200,)*5,
            )
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            # wait too long for trackTarget
            data = await harness.next_evt("atMountState", timeout=max_tracking_interval + 0.2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)
            await self.fault_to_enabled(harness)

            # try again, and this time send a trackTarget command
            # before waiting too long
            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            harness.remote.cmd_trackTarget.set(
                elevation=10,
                time=ATMCSSimulator.curr_tai(),
                trackId=20,  # arbitary
            )
            await harness.remote.cmd_trackTarget.start(timeout=1)

            # wait too long for trackTarget
            data = await harness.next_evt("atMountState", timeout=max_tracking_interval + 0.2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_Stopping)

            data = await harness.next_evt("atMountState", timeout=2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_stop_tracking_while_slewing(self):
        """Call stopTracking while tracking, before a slew is done.
        """
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            t0 = ATMCSSimulator.curr_tai()
            paths = dict(
                elevation=ATMCSSimulator.path.TPVAJ(t0=t0, p0=45),
                azimuth=ATMCSSimulator.path.TPVAJ(t0=t0, p0=100),
                nasmyth1RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=90),
                nasmyth2RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=-90),
            )
            trackId = 35  # arbitary

            t = t0 + 0.1  # offset is arbitrary but reasonable
            harness.remote.cmd_trackTarget.set(time=t, trackId=trackId)
            for axis_name, path in paths.items():
                pos, vel = path.pva(t)[0:2]
                kwargs = {
                    axis_name: pos,
                    f"{axis_name}Velocity": vel,
                }
                harness.remote.cmd_trackTarget.set(**kwargs)
            await harness.remote.cmd_trackTarget.start(timeout=1)

            await asyncio.sleep(0.2)

            await harness.remote.cmd_stopTracking.start(timeout=1)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_Stopping)

            await asyncio.sleep(0.2)  # give events time to arrive

            for evt_name in self.in_position_names:
                data = harness.get_evt(evt_name)
                if evt_name.startswith("m3"):
                    self.assertTrue(data.inPosition)
                else:
                    self.assertFalse(data.inPosition)

            data = await harness.next_evt("atMountState", timeout=2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            for actuator in harness.csc.actuators:
                self.assertEqual(actuator.kind(), ATMCSSimulator.path.Kind.Stopped)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_disable_while_slewing(self):
        """Call disable while tracking, before a slew is done.
        """
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            data = await harness.next_evt("atMountState", timeout=2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState", timeout=2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            t0 = ATMCSSimulator.curr_tai()
            paths = dict(
                elevation=ATMCSSimulator.path.TPVAJ(t0=t0, p0=45),
                azimuth=ATMCSSimulator.path.TPVAJ(t0=t0, p0=100),
                nasmyth1RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=90),
                nasmyth2RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=-90),
            )
            trackId = 35  # arbitary

            t = t0 + 0.1  # offset is arbitrary but reasonable
            harness.remote.cmd_trackTarget.set(time=t, trackId=trackId)
            for axis_name, path in paths.items():
                pos, vel = path.pva(t)[0:2]
                kwargs = {
                    axis_name: pos,
                    f"{axis_name}Velocity": vel,
                }
                harness.remote.cmd_trackTarget.set(**kwargs)
            await harness.remote.cmd_trackTarget.start(timeout=1)

            await asyncio.sleep(0.2)

            await harness.remote.cmd_disable.start(timeout=1)
            data = await harness.next_evt("atMountState", timeout=2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_Stopping)

            await asyncio.sleep(0.2)  # give events time to arrive

            for evt_name in self.in_position_names:
                data = harness.get_evt(evt_name)
                if evt_name.startswith("m3"):
                    self.assertTrue(data.inPosition)
                else:
                    self.assertFalse(data.inPosition)

            data = await harness.next_evt("atMountState", timeout=2)
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

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
