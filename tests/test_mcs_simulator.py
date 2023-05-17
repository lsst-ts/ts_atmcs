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

import asyncio
import contextlib
import logging
import typing
import unittest

from lsst.ts import atmcssimulator, tcpip

# Standard timeout in seconds.
TIMEOUT = 2


class McsSimulatorTestCase(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.log = logging.getLogger(type(self).__name__)

    @contextlib.asynccontextmanager
    async def create_mcs_simulator(
        self,
    ) -> typing.AsyncGenerator[atmcssimulator.McsSimulator, None]:
        async with atmcssimulator.McsSimulator() as simulator:
            await simulator.cmd_evt_server.start_task
            await simulator.telemetry_server.start_task
            yield simulator

    @contextlib.asynccontextmanager
    async def create_evt_cmd_client(
        self, simulator: atmcssimulator.McsSimulator
    ) -> typing.AsyncGenerator[tcpip.Client, None]:
        async with tcpip.Client(
            host=simulator.cmd_evt_server.host,
            port=simulator.cmd_evt_server.port,
            log=self.log,
            name="CmdEvtClient",
        ) as cmd_evt_client:
            await asyncio.wait_for(
                simulator.cmd_evt_server.connected_task, timeout=TIMEOUT
            )
            assert simulator.cmd_evt_server.connected
            assert cmd_evt_client.connected
            yield cmd_evt_client

    async def verify_command_response(
        self,
        client: tcpip.Client,
        ack: atmcssimulator.Ack,
        sequence_id: int,
    ) -> None:
        data = await client.read_json()
        assert atmcssimulator.CommandKey.ID in data
        assert atmcssimulator.CommandKey.SEQUENCE_ID in data
        assert data[atmcssimulator.CommandKey.ID] == ack
        assert data[atmcssimulator.CommandKey.SEQUENCE_ID] == sequence_id

    # TODO DM-39012: Improve this test after adding the simulator code.
    async def test_set_instrument_port(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_evt_cmd_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            port = 25000
            await cmd_evt_client.write_json(
                data={
                    atmcssimulator.CommandKey.ID: "cmd_setInstrumentPort",
                    atmcssimulator.CommandKey.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandKey.PORT: port,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

    # TODO DM-39012: Improve this test after adding the simulator code.
    async def test_start_tracking(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_evt_cmd_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    atmcssimulator.CommandKey.ID: "cmd_startTracking",
                    atmcssimulator.CommandKey.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandKey.VALUE: True,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

    # TODO DM-39012: Improve this test after adding the simulator code.
    async def test_stop_tracking(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_evt_cmd_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    atmcssimulator.CommandKey.ID: "cmd_stopTracking",
                    atmcssimulator.CommandKey.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandKey.VALUE: True,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

    # TODO DM-39012: Improve this test after adding the simulator code.
    async def test_track_target(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_evt_cmd_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    atmcssimulator.CommandKey.ID: "cmd_trackTarget",
                    atmcssimulator.CommandKey.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandKey.AZIMUTH: 0.0,
                    atmcssimulator.CommandKey.AZIMUTH_VELOCITY: 0.0,
                    atmcssimulator.CommandKey.ELEVATION: 0.0,
                    atmcssimulator.CommandKey.ELEVATION_VELOCITY: 0.0,
                    atmcssimulator.CommandKey.NASMYTH1_ROTATOR_ANGLE: 0.0,
                    atmcssimulator.CommandKey.NASMYTH1_ROTATOR_ANGLE_VELOCITY: 0.0,
                    atmcssimulator.CommandKey.NASMYTH2_ROTATOR_ANGLE: 0.0,
                    atmcssimulator.CommandKey.NASMYTH2_ROTATOR_ANGLE_VELOCITY: 0.0,
                    atmcssimulator.CommandKey.RA_DE_SYS: "ICRS",
                    atmcssimulator.CommandKey.TAI_TIME: 0.0,
                    atmcssimulator.CommandKey.TRACK_ID: 1,
                    atmcssimulator.CommandKey.TRACK_SYS: "sidereal",
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

    # TODO DM-39012: Improve this test after adding the simulator code.
    async def test_non_existing_command(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_evt_cmd_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    atmcssimulator.CommandKey.ID: "non-existing",
                    atmcssimulator.CommandKey.SEQUENCE_ID: sequence_id,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.NOACK,
                sequence_id=sequence_id,
            )

    # TODO DM-39012: Improve this test after adding the simulator code.
    async def test_skip_sequence_id(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_evt_cmd_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    atmcssimulator.CommandKey.ID: "cmd_startTracking",
                    atmcssimulator.CommandKey.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandKey.VALUE: True,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

            # Skip sequence_id == 2 so we expect a NOACK.
            sequence_id = 3
            await cmd_evt_client.write_json(
                data={
                    atmcssimulator.CommandKey.ID: "cmd_startTracking",
                    atmcssimulator.CommandKey.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandKey.VALUE: True,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=atmcssimulator.Ack.NOACK,
                sequence_id=sequence_id,
            )
