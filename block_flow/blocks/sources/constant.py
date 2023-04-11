from block_flow.blocks.block import Block
from block_flow.connections.signals import Signal


class Constant(Block):
    def __init__(self, value, sample_time: float = None, name: str = None) -> None:
        super().__init__(num_inputs=0, num_outputs=1, sample_time=sample_time, name=name)
        self.value = value

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t) -> None:
        self.outputs[0].data = (self.value)
