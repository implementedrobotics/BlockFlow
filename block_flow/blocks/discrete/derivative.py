from block_flow.blocks.block import Block
from block_flow.connections.signals import Signal


class Derivative(Block):
    def __init__(self, dt, name: str = None) -> None:
        super().__init__(1, 1)
        self.dt = dt
        self.prev_input = None

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        if self.prev_input is not None:
            self.outputs[0].data = (
                (self.inputs[0].data - self.prev_input) / self.dt)
        self.prev_input = self.inputs[0].data
