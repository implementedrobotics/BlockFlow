from block_flow.blocks.block import Block
from block_flow.connections.signal import Signal


class Integrator(Block):
    def __init__(self, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.prev_val = 0

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = (self.prev_val + self.inputs[0].data)
        self.prev_val = self.outputs[0].data
