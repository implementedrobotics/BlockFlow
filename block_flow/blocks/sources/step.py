from block_flow.blocks.block import Block
from block_flow.connections.port import OutputPort


class Step(Block):
    def __init__(self, T: float = 0, init_val: float = 0, step_val: float = 1, name: str = None) -> None:
        super().__init__(num_inputs=0, num_outputs=1, name=name)
        self.T = T
        self.init_val = init_val
        self.step_val = step_val

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [type(self.init_val)]))

        # Set Initial Output
        self.outputs[0].data = (self.init_value)

    def update(self, t: float) -> None:
        self.outputs[0].data = (self.step_val if t >=
                                self.T else self.init_val)
