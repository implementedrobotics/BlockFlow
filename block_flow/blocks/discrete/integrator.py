from block_flow.blocks.block import Block
from block_flow.connections.port import OutputPort, InputPort


class Integrator(Block):
    def __init__(self, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.prev_val = 0

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [float, int]))

        # Create a ports for the inputs
        self._add_input_port(0, InputPort(self, [float, int]))

    def update(self, t: float) -> None:
        self.outputs[0].data = (self.prev_val + self.inputs[0].data)
        self.prev_val = self.outputs[0].data
