from block_flow.blocks.block import Block
from block_flow.connections.port import OutputPort, InputPort


class DelayBlock(Block):
    pass


class ZeroOrderHold(DelayBlock):
    def __init__(self, sample_time, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.sample_time = sample_time
        self.last_sample_time = None

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, (float, int)))

        # Create a ports for the inputs
        self._add_input_port(0, InputPort(self, (float, int)))

        # TODO: Initial Value Param?
        self.outputs[0].data = 0

    def update(self, t: float) -> None:
        if self.last_sample_time is None or t >= self.last_sample_time + self.sample_time:
            self.outputs[0].data = self.inputs[0].data
            self.last_sample_time = t
