import asyncua
import asyncio
import logging

_logger = logging.getLogger(__name__)

class AbstractGlobalVarManager:

    def __init__(self):
        self._closing = False

    async def init(self):
        raise NotImplementedError

    async def add_input_var(self, input_var: str, default_value: bool | float | int):
        raise NotImplementedError

    async def add_output_var(self, output_var: str, default_value: bool | float | int):
        raise NotImplementedError

    async def read(self, var: str) -> bool | float | int:
        raise NotImplementedError

    async def write(self, var: str, value: bool | float | int):
        raise NotImplementedError

    def close(self, sig_num, stack):
        self._closing = True

    async def run(self):
        raise NotImplementedError





class OPCUAGlobalVarManager(AbstractGlobalVarManager):
    _my_obj: None | asyncua.Node

    def __init__(self):
        super().__init__()
        self.opcua_server = asyncua.Server()
        self._idx = 0
        self._input_vars: dict[str, asyncua.Node] = {}
        self._output_vars: dict[str, asyncua.Node] = {}
        self._my_obj = None


    async def init(self):
        await self.opcua_server.init()
        await self.opcua_server.set_endpoint("opc.tcp://0.0.0.0:4840/DT-conveyor/global")
        uri = "http://DT-conveyor.github.io"
        self._idx = await self.opcua_server.register_namespace(uri)
        self._my_obj= await self.opcua_server.nodes.objects.add_object(self._idx, "Smart_conveyor")

    async def add_input_var(self, input_var: str, default_value: bool | float | int):
        assert input_var not in self._output_vars
        self._input_vars[input_var] = await self._my_obj.add_variable(self._idx, input_var, default_value)
        await self._input_vars[input_var].set_writable()

    async def add_output_var(self, output_var: str, default_value: bool | float | int):
        assert output_var not in self._output_vars
        self._output_vars[output_var] = await self._my_obj.add_variable(self._idx, output_var, default_value)

    async def read(self, var: str) -> bool | float | int:
        if var in self._input_vars:
            var_node = self._input_vars[var]
        elif var in self._output_vars:
            var_node = self._output_vars[var]
        else:
            raise KeyError(var)
        return await var_node.read_value()

    async def write(self, var: str, value: bool | float | int):
        if var in self._output_vars:
            await self._output_vars[var].write_value(value)
        else:
            raise KeyError(var)

    def close(self, sig_num, stack):
        self._closing = True

    async def run(self):
        _logger.info("Initializing...")
        while not self._closing:
            with self.opcua_server:
                await asyncio.sleep(1.0)
        _logger.info("Closing...")

