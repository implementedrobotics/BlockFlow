from block_flow.connections.port import OutputPort, InputPort
from block_flow.blocks.block import Block

from functools import reduce
import operator


class Add(Block):

    def __init__(self, num_inputs: int = 2, operations: list = None, sample_time: float = None, name: str = None) -> None:
        super().__init__(num_inputs=num_inputs, num_outputs=1,
                         sample_time=sample_time, name=name)

        self.operations = operations

        if num_inputs < 2:
            raise ValueError("[Add]: num_inputs must be at least 2")

        # Default to addition if not specificied for all inputs
        if operations is None:
            self.operations = ["+"] * num_inputs

        assert len(
            self.operations) == num_inputs, "Number of operations must match number of inputs."

        for i, (operation) in enumerate(self.operations):
            if operation == "+":
                pass
            elif operation == "-":
                pass
            else:
                raise ValueError(
                    f"Invalid operation: {operation} at input index {i}")

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [float, int]))

        # Create a ports for the inputs
        for i in range(num_inputs):
            self._add_input_port(i, InputPort(self, [float, int]))

    def update(self, t: float) -> None:

        # Map operation symbols to their corresponding functions
        operation_mapping = {
            "+": operator.add,
            "-": operator.sub
        }

        result = 0
        for i, (operation, input) in enumerate(zip(self.operations, self.inputs)):
            try:
                operation_func = operation_mapping[operation]
            except KeyError:
                raise ValueError(
                    f"Invalid operation: {operation} at input index {i}")

            result = operation_func(result, input.data)

        self.outputs[0].data = result


class Div(Block):
    def __init__(self, num_inputs: int = 2, name: str = None) -> None:
        if num_inputs < 2:
            raise ValueError("[Mul]: num_inputs must be at least 2")

        super().__init__(num_inputs=num_inputs, num_outputs=1, name=name)

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [float, int]))

        # Create a ports for the inputs
        for i in range(num_inputs):
            self._add_input_port(i, InputPort(self, [float, int]))

    def update(self, t):

        output_data = reduce(
            operator.truediv, (input_signal.data for input_signal in self.inputs))

        self.outputs[0].data = output_data


class Mul(Block):
    def __init__(self, num_inputs=2, sample_time: float = None, name: str = None) -> None:
        if num_inputs < 2:
            raise ValueError("[Mul]: num_inputs must be at least 2")

        super().__init__(num_inputs=num_inputs, num_outputs=1,
                         sample_time=sample_time, name=name)

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [float, int]))

        # Create a ports for the inputs
        for i in range(num_inputs):
            self._add_input_port(i, InputPort(self, [float, int]))

    def update(self, t: float) -> None:

        output_data = reduce(
            operator.mul, (input_signal.data for input_signal in self.inputs))

        self.outputs[0].data = output_data


class Gain(Block):
    def __init__(self, gain: float, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.gain = gain

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [float, int]))

        # Create a ports for the inputs
        self._add_input_port(0, InputPort(self, [float, int]))

    def update(self, t: float) -> None:
        self.outputs[0].data = (self.inputs[0].data * self.gain)
