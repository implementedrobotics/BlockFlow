import mujoco
import math
import numpy as np
from block_flow.connections.port import OutputPort, InputPort
from block_flow.blocks.block import Block


class MujocoPlant(Block):
    def __init__(self, model_path: str | None = None, model: mujoco.MjModel | None = None, data: mujoco.MjData | None = None, x_0: np.ndarray | None = None, sample_time: float = None, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, sample_time=sample_time, name=name)

        if model_path is not None:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
        elif model is not None and data is not None:
            self.model = model
            self.data = data
        else:
            raise ValueError(
                "Either a model_path or both model and data parameters must be provided.")

        # TODO: List of Pos/Vel indices for the saved state of the plan
        # Update the initial state vector
        self._x_0 = x_0

        # If no initial state is provided, use the default state from the model
        if self._x_0 is None:
            self._x_0 = np.concatenate((self.data.qpos, self.data.qvel))

        # Create the state vector
        self._x = self._x_0

        # Create a port for the output
        self._add_output_port(0, OutputPort(
            self, data_types=(np.ndarray), name="Y"))

        # Create a ports for the inputs
        self._add_input_port(0, InputPort(
            self, data_types=(np.ndarray)))

        # Set Initial Output
        self.outputs[0].data = (self._x)

    def update(self, t) -> None:

        # Get control vector
        u = self.inputs[0].data

        # Apply Control Vector
        self.data.ctrl[:] = u

        # Step Simulation
        mujoco.mj_step(self.model, self.data)

        # Update the state vector
        self._x = np.concatenate([self.data.qpos[:], self.data.qvel[:]])

        # TODO: Call f(x) and g(x) to get the next state

        # Set Output
        self.outputs[0].data = self._x

        print(self._x)

    def f(self, x, u) -> None:
        pass
