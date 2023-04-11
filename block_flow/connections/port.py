from typing import TYPE_CHECKING
from typing import Type

if TYPE_CHECKING:
    from block_flow.blocks.block import Block


class Port:
    def __init__(self, block: "Block", data_type: Type, name: str = None):

        # Block to which the signal belongs
        self.block = block

        # Data Type
        self.data_type = data_type

        # Port name
        self.name = name

        # Port ID
        self.port_id = None

        # Signal value
        self._data = data_type()


class InputPort(Port):
    def __init__(self, block: "Block", data_type: Type, name: str = None):
        super().__init__(block, data_type, name)

    def get_input(self):
        return self.signal.data


class OutputPort(Port):
    def __init__(self, block: "Block", data_type: Type, name: str = None):
        super().__init__(block, data_type, name)

        # List of connected signals
        self.connections = []

    def set_output(self, data):
        self.signal.data = data
