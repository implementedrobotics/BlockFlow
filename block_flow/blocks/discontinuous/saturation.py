
from block_flow.blocks.block import Block
from block_flow.connections.signals import Signal


class Saturation(Block):
    def __init__(self, min_val, max_val, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.min_val = min_val
        self.max_val = max_val

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = (
            min(max(self.inputs[0].data, self.min_val), self.max_val))
