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

from __future__ import annotations

import logging
import types
import typing

import jsonschema
from lsst.ts import tcpip

from .enums import Ack, CommandKey
from .mcs_server_simulator import McsServerSimulator
from .schemas.registry import registry

__all__ = ["McsSimulator"]

CMD_ITEMS_TO_IGNORE = frozenset({CommandKey.ID, CommandKey.VALUE})


class McsSimulator:
    def __init__(self) -> None:
        self.log = logging.getLogger(type(self).__name__)
        self.cmd_evt_server = McsServerSimulator(
            host=tcpip.LOCALHOST_IPV4,
            port=5000,
            log=self.log,
            dispatch_callback=self.cmd_evt_dispatch_callback,
            name="CmdEvtMcsServer",
        )
        self.telemetry_server = McsServerSimulator(
            host=tcpip.LOCALHOST_IPV4,
            port=6000,
            log=self.log,
            dispatch_callback=self.telemetry_dispatch_callback,
            name="TelemetryMcsServer",
        )

        # Keep track of the sequence_id as commands come dripping in. The
        # sequence ID should raise monotonally without gaps. If a gap is seen,
        # a NOACK should be returned.
        self.last_sequence_id = 0

        # Dict of command: function.
        self.dispatch_dict: dict[str, typing.Callable] = {
            "setInstrumentPort": self.set_instrument_port,
            "startTracking": self.start_tracking,
            "stopTracking": self.stop_tracking,
            "trackTarget": self.track_target,
        }

    async def cmd_evt_dispatch_callback(self, data: typing.Any) -> None:
        data_ok = await self.verify_data(data=data)
        if not data_ok:
            await self.write_noack_response(sequence_id=data[CommandKey.SEQUENCE_ID])
            return

        await self.write_ack_response(sequence_id=data[CommandKey.SEQUENCE_ID])

        cmd = data[CommandKey.ID].replace("cmd_", "")
        func = self.dispatch_dict[cmd]
        kwargs = {
            key: value for key, value in data.items() if key not in CMD_ITEMS_TO_IGNORE
        }
        await func(**kwargs)

    async def telemetry_dispatch_callback(self, data: typing.Any) -> None:
        pass

    async def verify_data(self, data: dict[str, typing.Any]) -> bool:
        """Verify the format and values of the data.

        The format of the data is described at
        https://github.com/lsst-ts/ts_labview_tcp_json
        as well as in the JSON schemas in the schemas directory.

        Parameters
        ----------
        data : `dict` of `any`
            The dict to be verified.

        Returns
        -------
        bool:
            Whether the data follows the correct format and has the correct
            contents or not.
        """
        if CommandKey.ID not in data or CommandKey.SEQUENCE_ID not in data:
            self.log.error(f"Received invalid {data=}. Ignoring.")
            return False
        payload_id = data[CommandKey.ID].replace("cmd_", "command_")
        if payload_id not in registry:
            self.log.error(f"Unknown command in {data=}.")
            return False

        sequence_id = data[CommandKey.SEQUENCE_ID]
        if self.last_sequence_id == 0:
            self.last_sequence_id = sequence_id
        else:
            if sequence_id - self.last_sequence_id != 1:
                return False

        json_schema = registry[payload_id]
        try:
            jsonschema.validate(data, json_schema)
        except jsonschema.ValidationError as e:
            self.log.exception("Validation failed.", e)
            return False
        return True

    async def set_instrument_port(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        # TODO DM-39012: Add simulator code.
        await self.write_success_response(sequence_id=sequence_id)

    async def start_tracking(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        # TODO DM-39012: Add simulator code.
        await self.write_success_response(sequence_id=sequence_id)

    async def stop_tracking(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        # TODO DM-39012: Add simulator code.
        await self.write_success_response(sequence_id=sequence_id)

    async def track_target(
        self, sequence_id: int, **kwargs: dict[str, typing.Any]
    ) -> None:
        # TODO DM-39012: Add simulator code.
        await self.write_success_response(sequence_id=sequence_id)

    async def write_response(self, response: str, sequence_id: int) -> None:
        data = {CommandKey.ID: response, CommandKey.SEQUENCE_ID: sequence_id}
        await self.cmd_evt_server.write_json(data=data)

    async def write_ack_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.ACK, sequence_id)

    async def write_fail_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.FAIL, sequence_id)

    async def write_noack_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.NOACK, sequence_id)

    async def write_success_response(self, sequence_id: int) -> None:
        await self.write_response(Ack.SUCCESS, sequence_id)

    def __enter__(self) -> None:
        # This class only implements an async context manager.
        raise NotImplementedError("Use 'async with' instead.")

    def __exit__(
        self,
        type: typing.Type[BaseException],
        value: BaseException,
        traceback: types.TracebackType,
    ) -> None:
        # __exit__ should exist in pair with __enter__ but never be executed.
        raise NotImplementedError("Use 'async with' instead.")

    async def __aenter__(self) -> McsSimulator:
        return self

    async def __aexit__(
        self,
        type: typing.Type[BaseException],
        value: BaseException,
        traceback: types.TracebackType,
    ) -> None:
        await self.cmd_evt_server.close()
        await self.telemetry_server.close()
