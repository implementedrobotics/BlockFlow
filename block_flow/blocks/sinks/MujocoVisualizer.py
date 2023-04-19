import mujoco
import mujoco_viewer
import math
import numpy as np
from block_flow.connections.port import OutputPort, InputPort
from block_flow.blocks.block import Block
import multiprocessing


class MuJoCoVisualizationBlock(Block):
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

        # Create a ports for the inputs
        self._add_input_port(0, InputPort(
            self, data_types=(np.ndarray)))

        # Create a lock for protecting access to the shared data (input signal)
        self.lock = multiprocessing.Lock()

        self.viewer = None
        self.input_test = np.ndarray([])

        # Create a separate thread for visualization
        self.vis_process = multiprocessing.Process(target=self._visualize)
        self.vis_process.start()

    def _visualize(self):

        # Create the viewer for visualization
        if self.viewer is None:
            self.viewer = mujoco_viewer.MujocoViewer(
                self.model, self.data, width=1200, height=1200)
            self.viewer._render_every_frame = False

        while self.viewer.is_alive:
            # with self.lock:
            #     # Update the MuJoCo simulation with the input data
            #     self._update_simulation(self.inputs[0].data)

            self.data.ctrl[:] = 1
            mujoco.mj_step(self.model, self.data)

            self.viewer.add_marker(pos=np.array([0, 0, 1]),
                                   label="ORIGIN")
            # Render the updated simulation in the viewer
            self.viewer.render()

    def _update_simulation(self, data):
        # Update the MuJoCo simulation with the input data
        # You may need to modify this method based on the structure of your input data
        # TODO: Copy State to internal mujoco state
        pass

    def update(self, t):
        # with self.lock:
        # Update the input data safely using the lock
        #    self.input_test = self.inputs[0].data
        pass

    def _get_input_data(self, t):
        # Generate or fetch input data for the simulation
        # You may need to modify this method based on your specific use case

        pass
