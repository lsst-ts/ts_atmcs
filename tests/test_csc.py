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
import time
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
            raise RuntimeError(f"Cound not get data for event {name}") from e

    def get_evt(self, name):
        try:
            evt = getattr(self.remote, f"evt_{name}")
            return evt.get()
        except Exception as e:
            raise RuntimeError(f"Cound not get data for event {name}") from e


class CscTestCase(unittest.TestCase):
    def setUp(self):
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
            harness.csc.configure(
                vmax=(100,)*5,
                amax=(200,)*5,
            )
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            # cannot send trackTarget while tracking disabled
            harness.remote.cmd_trackTarget.set(
                azimuthDirection=SALPY_ATMCS.ATMCS_shared_AzimuthDirection_ClockWise,
                elevation=10,
                time=time.time(),
                trackId=137)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)

            # enable tracking and try again; this time it should work
            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)
            await harness.remote.cmd_trackTarget.start(timeout=1)

            # azimuth must be in range 0, 360 (regardless of the time)
            harness.remote.cmd_trackTarget.set(azimuth=-0.000001)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            harness.remote.cmd_trackTarget.set(
                azimuth=360.00001,
                time=time.time())
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            # azimuthDirection must be 1 or 2
            harness.remote.cmd_trackTarget.set(
                azimuth=0,
                azimuthDirection=0,
                time=time.time())
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            harness.remote.cmd_trackTarget.set(
                azimuth=0,
                azimuthDirection=3,
                time=time.time())
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

            await harness.remote.cmd_startTracking.start(timeout=2)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingEnabled)

            # a target that is (way) out of bounds at the specified time
            harness.remote.cmd_trackTarget.set(
                elevation=85,
                elevationVelocity=2,
                azimuthDirection=SALPY_ATMCS.ATMCS_shared_AzimuthDirection_ClockWise,
                time=time.time() - 10)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_trackTarget.start(timeout=1)
            data = await harness.next_evt("atMountState")
            self.assertEqual(data.state, SALPY_ATMCS.ATMCS_shared_AtMountState_TrackingDisabled)

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

            t0 = time.time()

            await asyncio.sleep(0.2)

            # attempts to start tracking should fail while M3 is moving
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_startTracking.start()

            actuator = harness.csc.actuators[ATMCSSimulator.Axis.M3]
            curr_pos, curr_vel = actuator.curr.pva(time.time())[0:2]
            self.assertNotEqual(curr_vel, 0)

            data = await harness.next_evt("m3State", timeout=5)
            print(f"test_set_instrument_port M3 rotation took {time.time() - t0:0.2f} sec")
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

            t0 = time.time()
            paths = dict(
                elevation=ATMCSSimulator.path.TPVAJ(t0=t0, p0=6, v0=0.001),
                azimuth=ATMCSSimulator.path.TPVAJ(t0=t0, p0=5, v0=-0.001),
                nasmyth1RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=1, v0=-0.001),
                nasmyth2RotatorAngle=ATMCSSimulator.path.TPVAJ(t0=t0, p0=-2, v0=0.001),
            )
            trackId = 20  # arbitary
            while True:
                t = time.time() + 0.1  # offset is arbitrary but reasonable
                harness.remote.cmd_trackTarget.set(time=t, trackId=trackId)
                for axis_name, path in paths.items():
                    pos, vel = path.pva(t)[0:2]
                    kwargs = {
                        axis_name: pos,
                        f"{axis_name}Velocity": vel,
                    }
                    harness.remote.cmd_trackTarget.set(**kwargs)
                # either wrap will do; pick one
                harness.remote.cmd_trackTarget.set(
                    azimuthDirection=SALPY_ATMCS.ATMCS_shared_AzimuthDirection_CounterClockWise)
                await harness.remote.cmd_trackTarget.start(timeout=1)

                data = harness.remote.evt_allAxesInPosition.get()
                if data.inPosition:
                    break

                if time.time() - t0 > 5:
                    raise self.fail("Timed out waiting for slew to finish")

                await asyncio.sleep(0.5)

            print(f"test_track slew took {time.time() - t0:0.2f} sec")

            for evt_name in self.in_position_names:
                if evt_name.startswith("m3"):
                    continue  # already was in position
                data = await harness.next_evt(evt_name)
                self.assertTrue(data.inPosition)

            await harness.remote.cmd_stopTracking.start(timeout=1)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_stop_tracking(self):
        """Call stopTracking before a slew is done.
        """
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            await harness.remote.cmd_startTracking.start(timeout=2)

            t0 = time.time()
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
            harness.remote.cmd_trackTarget.set(
                azimuthDirection=SALPY_ATMCS.ATMCS_shared_AzimuthDirection_ClockWise)
            await harness.remote.cmd_trackTarget.start(timeout=1)

            await asyncio.sleep(0.2)

            await harness.remote.cmd_stopTracking.start(timeout=1)

            await asyncio.sleep(0.2)  # give events time to arrive

            for evt_name in self.in_position_names:
                data = harness.get_evt(evt_name)
                if evt_name.startswith("m3"):
                    self.assertTrue(data.inPosition)
                else:
                    self.assertFalse(data.inPosition)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_disable_while_tracking(self):
        """Call disable before a slew is done.
        """
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            await harness.remote.cmd_startTracking.start(timeout=2)

            t0 = time.time()
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
            harness.remote.cmd_trackTarget.set(
                azimuthDirection=SALPY_ATMCS.ATMCS_shared_AzimuthDirection_ClockWise)
            await harness.remote.cmd_trackTarget.start(timeout=1)

            await asyncio.sleep(0.2)

            await harness.remote.cmd_disable.start(timeout=1)

            await asyncio.sleep(0.2)  # give events time to arrive

            for evt_name in self.in_position_names:
                data = harness.get_evt(evt_name)
                if evt_name.startswith("m3"):
                    self.assertTrue(data.inPosition)
                else:
                    self.assertFalse(data.inPosition)

        asyncio.get_event_loop().run_until_complete(doit())


if __name__ == "__main__":
    unittest.main()
