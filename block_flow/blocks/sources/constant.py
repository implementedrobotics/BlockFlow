from block_flow.blocks.block import Block
from block_flow.connections.port import OutputPort


class Constant(Block):
    def __init__(self, value, sample_time: float = None, name: str = None) -> None:
        super().__init__(num_inputs=0, num_outputs=1, sample_time=sample_time, name=name)
        self.value = value

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [type(value)]))

        # Set Initial Output
        self.outputs[0].data = (self.value)

    def update(self, t) -> None:

        # Set Output
        self.outputs[0].data = (self.value)
