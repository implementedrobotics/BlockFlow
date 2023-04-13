from typing import TYPE_CHECKING
from typing import Type, Union, Tuple

import numpy as np

if TYPE_CHECKING:
    from block_flow.blocks.block import Block

ALLOWED_DATA_TYPES = (int, float, bool, np.ndarray)
SCALAR_DATA_TYPES = (int, float, bool)
VECTOR_DATA_TYPES = (np.ndarray,)
# GENERIC_DATA_TYPES


class PortData:
    def __init__(self, value):
        self._value = None

    @property
    def data(self):
        return self._value

    @data.setter
    def data(self, value):
        self._value = value


class Port:
    def __init__(self, block: "Block", data_types: Union[Type, Tuple[Type]], name: str = None):

        # Block to which the signal belongs
        self.block = block

        # Data Type
        self.data_types = data_types

        # Port name
        self.name = name

        # Port ID
        self.port_id = None

        # Signal value
        self._port_data = PortData(value=data_types[0]())

        # Connected?
        self.connected = False


class InputPort(Port):
    def __init__(self, block: "Block", data_types: list[Type[Union[int, float, bool, np.ndarray]]], name: str = None):
        super().__init__(block, data_types, name)

    @property
    def data(self):
        return self._port_data._value

    @data.setter
    def data(self, value):
        if not any(isinstance(value, data_type) for data_type in self.data_types):
            raise TypeError(
                f"Data must be one of the following types: {', '.join(str(t) for t in self.data_types)}")
        self._port_data._value = value


class OutputPort(Port):
    def __init__(self, block: "Block", data_types: list[Type[Union[int, float, bool, np.ndarray]]], name: str = None):
        super().__init__(block, data_types, name)

        # List of connected signals
        self.connections = []

    def _connect(self, dest) -> bool:
        dest._port_data = self._port_data

        return True

    @property
    def data(self):
        return self._port_data._value

    @data.setter
    def data(self, value):
        if not any(isinstance(value, data_type) for data_type in self.data_types):
            raise TypeError(
                f"Data must be one of the following types: {', '.join(str(t) for t in self.data_types)}")
        self._port_data._value = value
