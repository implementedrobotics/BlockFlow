from typing import TYPE_CHECKING
from typing import Type, Union, Tuple

import numpy as np

if TYPE_CHECKING:
    from block_flow.blocks.block import Block

ALLOWED_DATA_TYPES = (int, float, bool, np.ndarray)
SCALAR_DATA_TYPES = (int, float, bool)
VECTOR_DATA_TYPES = (np.ndarray,)
# GENERIC_DATA_TYPES


class Signal:
    def __init__(self, value, data_types: Union[Type, Tuple[Type]], names=None):

        self._value = value

        if isinstance(data_types, tuple):
            self.data_types = data_types
        else:
            self.data_types = (data_types,)

        self.names = names if names else []

    @property
    def data(self):
        return self._value

    @data.setter
    def data(self, value):
        self._value = value

    def __str__(self):
        return f"Signal(data={self.data}, names={self.names})"


# class PortData:
#     def __init__(self, value=None):
#         self._value = value

#     @property
#     def data(self):
#         return self._value

#     @data.setter
#     def data(self, value):
#         self._value = value


class Port:
    def __init__(self, block: "Block", data_types: Union[Type, Tuple[Type]], signal=None, name: str = None):

        # Block to which the signal belongs
        self.block = block

        # Ensure data_type is always a tuple, even when a single type is provided
        if isinstance(data_types, tuple):
            self.data_types = data_types
        else:
            self.data_types = (data_types,)

        # Signals
        self.signal = signal if signal else Signal(None, data_types=data_types)

        # Port name
        self.name = name

        # Port ID
        self.port_id = None

        # Signal value
        # self._port_data = PortData(value=None)

        # Connected?
        self.connected = False

    @staticmethod
    def _are_ports_compatible(output_port: "OutputPort", input_port: "InputPort") -> bool:

        return any(
            isinstance(output_port.data, data_type)
            for data_type in input_port.data_types
        )

# TODO: Set Signal Names
    def set_signal_names(self, names: list[str]):
        pass


class InputPort(Port):
    def __init__(self, block: "Block", data_types: Union[Type, Tuple[Type]], signal=None, name: str = None):
        super().__init__(block, data_types, signal, name)

    @property
    def data(self):
        return self.signal.data

    @data.setter
    def data(self, value):
        if not any(isinstance(value, data_type) for data_type in self.data_types):
            raise TypeError(
                f"Data must be one of the following types: {', '.join(str(t) for t in self.data_types)}")
        self.signal._value = value


class OutputPort(Port):
    def __init__(self, block: "Block", data_types: Union[Type, Tuple[Type]], signal=None, name: str = None):
        super().__init__(block, data_types, signal, name)

        # List of connected signals
        self.connections = []

    def _connect(self, dest) -> bool:

        if not Port._are_ports_compatible(self, dest):
            raise TypeError(
                f"Cannot connect {self.block.name} to {dest.block.name}. Port types are not compatible.")

        dest.signal = self.signal

        return True

    @property
    def data(self):
        return self.signal._value

    @data.setter
    def data(self, value):
        if not any(isinstance(value, data_type) for data_type in self.data_types):
            raise TypeError(
                f"Data must be one of the following types: {', '.join(str(t) for t in self.data_types)}")
        self.signal._value = value
