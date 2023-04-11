from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from block_flow.blocks.block import Block


class Signal:
    def __init__(self, block: "Block", name: str = None) -> None:

        # Signal name
        self.name = name

        # Port ID
        self.port_id = None

        # Block to which the signal belongs
        self.block = block

        # Signal value
        self._data = 0

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, value):
        self._data = value
