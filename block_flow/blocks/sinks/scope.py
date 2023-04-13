from block_flow.blocks.block import Block
from block_flow.connections.port import InputPort

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('QtAgg')


class Scope(Block):
    def __init__(self, num_inputs: int = 1, max_time_steps: int = 100, name: str = None) -> None:
        super().__init__(num_inputs=num_inputs, num_outputs=0, name=name)
        self.max_time_steps = max_time_steps
        self.time_data = []
        self.input_data = []
        self.figure = plt.figure()
        self.axis = self.figure.add_subplot(num_inputs, 1, 1)
        self.axis.set_title('Scope')
        self.axis.set_xlabel('Time')
        self.axis.set_ylabel('Input Data')

        # Create a ports for the inputs
        for i in range(num_inputs):
            self._add_input_port(i, InputPort(self, (float, int)))

    def update(self, t: float) -> None:

        self.time_data.append(t)
        self.input_data.append(self.inputs[0].data)

        if len(self.time_data) > self.max_time_steps:
            self.time_data.pop(0)
            self.input_data.pop(0)

        self.axis.clear()
        self.axis.plot(self.time_data, self.input_data)
        plt.pause(0.001)
        self.axis.set_title('Scope')
        self.axis.set_xlabel('Time')
        self.axis.set_ylabel('Input Data')

    def view(self) -> None:
        plt.show()
        # print(f"self.time_data: {self.time_data}")
        # print(f"self.input_data: {self.input_data}")
