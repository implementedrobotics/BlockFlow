from block_flow.blocks.block import Block
from block_flow.connections.port import OutputPort, InputPort


class Derivative(Block):
    def __init__(self, dt, name: str = None) -> None:
        super().__init__(1, 1)
        self.dt = dt
        self.prev_input = None

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [float, int]))

        # Create a ports for the inputs
        self._add_input_port(0, InputPort(self, [float, int]))

    def update(self, t: float) -> None:
        if self.prev_input is not None:
            self.outputs[0].data = (
                (self.inputs[0].data - self.prev_input) / self.dt)
        self.prev_input = self.inputs[0].data
