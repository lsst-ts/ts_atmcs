import asyncio
import typing

from lsst.ts import atmcssimulator, tcpip


class McsServerSimulatorTestCase(tcpip.BaseOneClientReadLoopServerTestCase):
    async def asyncSetUp(self) -> None:
        await super().asyncSetUp()
        self.data_event = asyncio.Event()
        self.data: typing.Any | None = None

    async def create_server(self) -> tcpip.OneClientReadLoopServer:
        return atmcssimulator.McsServerSimulator(
            host=tcpip.LOCAL_HOST,
            port=0,
            log=self.log,
            dispatch_callback=self.dispatch_callback,
            name="McsServerSimulator",
        )

    async def dispatch_callback(self, data: typing.Any) -> None:
        """Dispatch callback method for the McsServerSimulator to use.

        Parameters
        ----------
        data : `any`
            The data read by the McsServerSimulator.
        """
        self.data = data
        self.data_event.set()

    async def test_read_and_dispatch(self) -> None:
        async with self.create_server_and_client():
            self.data_event.clear()
            data = "Test."
            await self.client.write_json(data=data)
            await self.data_event.wait()
            assert self.data == data
