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

import asyncio
import contextlib
import logging
import typing
import unittest

import jsonschema
from lsst.ts import atmcssimulator, attcpip, tcpip, utils

# Standard timeout in seconds.
TIMEOUT = 2

EVENTS_TO_NOT_EXPECT = {
    atmcssimulator.Event.DETAILEDSTATE,
    atmcssimulator.Event.M3PORTSELECTED,
    atmcssimulator.Event.POSITIONLIMITS,
    atmcssimulator.Event.TARGET,
}
EVENTS_TO_EXPECT = set(atmcssimulator.Event) - EVENTS_TO_NOT_EXPECT


class McsSimulatorTestCase(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.log = logging.getLogger(type(self).__name__)

    @contextlib.asynccontextmanager
    async def create_mcs_simulator(
        self,
    ) -> typing.AsyncGenerator[atmcssimulator.McsSimulator, None]:
        async with atmcssimulator.McsSimulator(
            host=tcpip.LOCALHOST_IPV4, cmd_evt_port=5000, telemetry_port=6000
        ) as simulator:
            await simulator.cmd_evt_server.start_task
            await simulator.telemetry_server.start_task
            await simulator.configure()
            yield simulator

    @contextlib.asynccontextmanager
    async def create_cmd_evt_client(
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
            await self.verify_event(
                client=cmd_evt_client, evt_name=atmcssimulator.Event.POSITIONLIMITS
            )
            await self.verify_almost_all_events(client=cmd_evt_client)
            yield cmd_evt_client

    @contextlib.asynccontextmanager
    async def create_telemetry_client(
        self, simulator: atmcssimulator.McsSimulator
    ) -> typing.AsyncGenerator[tcpip.Client, None]:
        async with tcpip.Client(
            host=simulator.telemetry_server.host,
            port=simulator.telemetry_server.port,
            log=self.log,
            name="TelemetryClient",
        ) as telemetry_client:
            await asyncio.wait_for(
                simulator.telemetry_server.connected_task, timeout=TIMEOUT
            )
            assert simulator.telemetry_server.connected
            assert telemetry_client.connected
            yield telemetry_client

    async def verify_event(
        self,
        client: tcpip.Client,
        evt_name: str,
    ) -> None:
        data = await client.read_json()
        assert "id" in data
        assert data["id"] == evt_name

    async def verify_almost_all_events(self, client: tcpip.Client) -> None:
        for i in range(len(EVENTS_TO_EXPECT)):
            data = await client.read_json()
            # No need for asserts here. If the data id is not present in
            # registry or the validation of the schema fails, the test will
            # fail as well.
            json_schema = attcpip.registry[
                f"logevent_{data['id'].removeprefix('evt_')}"
            ]
            jsonschema.validate(data, json_schema)

    async def verify_command_response(
        self,
        client: tcpip.Client,
        ack: attcpip.Ack,
        sequence_id: int,
    ) -> None:
        data = await client.read_json()
        assert attcpip.CommonCommandArgument.ID in data
        while "evt" in data[attcpip.CommonCommandArgument.ID]:
            data = await client.read_json()
        assert attcpip.CommonCommandArgument.SEQUENCE_ID in data
        assert data[attcpip.CommonCommandArgument.ID] == ack
        assert data[attcpip.CommonCommandArgument.SEQUENCE_ID] == sequence_id

    async def test_set_instrument_port(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ) as cmd_evt_client:
            # First use a valid port number and verify that all responses are
            # as expected, including the SUCCESS response.
            sequence_id = 1
            port = 1
            await cmd_evt_client.write_json(
                data={
                    attcpip.CommonCommandArgument.ID: "cmd_setInstrumentPort",
                    attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandArgument.PORT: port,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_event(
                client=cmd_evt_client, evt_name=atmcssimulator.Event.M3PORTSELECTED
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

            # Repeat with an invalid port number and verify that all responses
            # are as expected, including the FAIL response.
            sequence_id = 2
            port = 25
            await cmd_evt_client.write_json(
                data={
                    attcpip.CommonCommandArgument.ID: "cmd_setInstrumentPort",
                    attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandArgument.PORT: port,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.FAIL,
                sequence_id=sequence_id,
            )

    async def verify_start_tracking(self, client: tcpip.Client) -> None:
        sequence_id = 1
        await client.write_json(
            data={
                attcpip.CommonCommandArgument.ID: "cmd_startTracking",
                attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                attcpip.CommonCommandArgument.VALUE: True,
            }
        )
        await self.verify_command_response(
            client=client, ack=attcpip.Ack.ACK, sequence_id=sequence_id
        )
        await self.verify_almost_all_events(client=client)
        await self.verify_command_response(
            client=client, ack=attcpip.Ack.SUCCESS, sequence_id=sequence_id
        )

    async def test_start_tracking(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ) as cmd_evt_client:
            await self.verify_start_tracking(client=cmd_evt_client)

    async def test_stop_tracking(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    attcpip.CommonCommandArgument.ID: "cmd_stopTracking",
                    attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                    attcpip.CommonCommandArgument.VALUE: True,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_almost_all_events(client=cmd_evt_client)
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

    async def test_track_target(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ) as cmd_evt_client:
            await self.verify_start_tracking(client=cmd_evt_client)

            # sequence_id == 1 was used by the start_tracking command.
            sequence_id = 2
            await cmd_evt_client.write_json(
                data={
                    attcpip.CommonCommandArgument.ID: "cmd_trackTarget",
                    attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                    atmcssimulator.CommandArgument.AZIMUTH: 0.0,
                    atmcssimulator.CommandArgument.AZIMUTH_VELOCITY: 0.0,
                    atmcssimulator.CommandArgument.ELEVATION: 10.0,
                    atmcssimulator.CommandArgument.ELEVATION_VELOCITY: 0.0,
                    atmcssimulator.CommandArgument.NASMYTH1_ROTATOR_ANGLE: 0.0,
                    atmcssimulator.CommandArgument.NASMYTH1_ROTATOR_ANGLE_VELOCITY: 0.0,
                    atmcssimulator.CommandArgument.NASMYTH2_ROTATOR_ANGLE: 0.0,
                    atmcssimulator.CommandArgument.NASMYTH2_ROTATOR_ANGLE_VELOCITY: 0.0,
                    atmcssimulator.CommandArgument.RA_DE_SYS: "ICRS",
                    atmcssimulator.CommandArgument.TAI_TIME: utils.current_tai(),
                    atmcssimulator.CommandArgument.TRACK_ID: 1,
                    atmcssimulator.CommandArgument.TRACK_SYS: "sidereal",
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_event(
                client=cmd_evt_client, evt_name=atmcssimulator.Event.TARGET
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

    async def test_non_existing_command(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    attcpip.CommonCommandArgument.ID: "non-existing",
                    attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.NOACK,
                sequence_id=sequence_id,
            )

    async def test_skip_sequence_id(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ) as cmd_evt_client:
            sequence_id = 1
            await cmd_evt_client.write_json(
                data={
                    attcpip.CommonCommandArgument.ID: "cmd_startTracking",
                    attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                    attcpip.CommonCommandArgument.VALUE: True,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.ACK,
                sequence_id=sequence_id,
            )
            await self.verify_almost_all_events(client=cmd_evt_client)
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.SUCCESS,
                sequence_id=sequence_id,
            )

            # Skip sequence_id == 2 so we expect a NOACK.
            sequence_id = 3
            await cmd_evt_client.write_json(
                data={
                    attcpip.CommonCommandArgument.ID: "cmd_stopTracking",
                    attcpip.CommonCommandArgument.SEQUENCE_ID: sequence_id,
                    attcpip.CommonCommandArgument.VALUE: True,
                }
            )
            await self.verify_command_response(
                client=cmd_evt_client,
                ack=attcpip.Ack.NOACK,
                sequence_id=sequence_id,
            )

    async def test_update_events(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ) as cmd_evt_client:
            await simulator.configure()
            await simulator.update_events()
            await self.verify_almost_all_events(client=cmd_evt_client)

    async def test_update_telemetry(self) -> None:
        async with self.create_mcs_simulator() as simulator, self.create_cmd_evt_client(
            simulator
        ), self.create_telemetry_client(simulator) as telemetry_client:
            # No need to call ``simulator.update_telemetry`` explicitly since
            # connecting with a cmd_evt_client starts the event and telemetry
            # loop.
            for _ in atmcssimulator.Telemetry:
                data = await telemetry_client.read_json()
                # No need for asserts here. If the data id is not present in
                # registry or the validation of the schema fails, the test will
                # fail as well.
                json_schema = attcpip.registry[f"{data['id']}"]
                jsonschema.validate(data, json_schema)
