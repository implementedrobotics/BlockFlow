from block_flow.blocks.block import Block
from block_flow.connections.port import OutputPort, InputPort

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from block_flow.blocks.subsystem.subsystem import SubSystemBlock


class SourceBlock(Block):
    def __init__(self, system_parent: "SubSystemBlock" = None, port_id=0, name: str = None):
        super().__init__(num_inputs=1, num_outputs=1, name=name)

        if port_id < 0 or port_id is None:
            raise ValueError("[Source Block]: Invalid Port Id (must be >= 0)")

        # Store a reference to the parent system
        self.system_parent = system_parent

        # Signal Port ID
        self.port_id = port_id

        # Create a port for the output
        self._add_output_port(0, OutputPort(self, [float, int]))

        # Create a port for the input
        self._add_input_port(0, InputPort(self, [float, int]))

    def set_parent(self, system_parent: "SubSystemBlock") -> None:
        self.system_parent = system_parent

    def update(self, t: float) -> None:
        if (self.inputs[0] is None):
            raise ReferenceError(
                f"[Source Block]: {self.name} - ERROR Input is NOT connected to a signal (port_id={self.port_id})")

        if self.inputs[0].connected:
            self.outputs[0].data = self.inputs[0].data
        else:
            print(
                f"[Source Block]: {self.name} - (port_id={self.port_id}) (connected={self.inputs[0].connected}) (data={self.inputs[0].data})")
