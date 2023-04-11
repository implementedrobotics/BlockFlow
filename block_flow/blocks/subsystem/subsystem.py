from block_flow.blocks.block import Block

from block_flow.blocks.subsystem.source import SourceBlock
from block_flow.blocks.subsystem.sink import SinkBlock

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from block_flow.systems.system import System


class SubSystemBlock(Block):
    def __init__(self, sub_system: "System", name: str = None) -> None:
        num_inputs = 0
        num_outputs = 0

        source_port_ids = set()
        sink_port_ids = set()

        # Count the number of source and sink blocks in the nested system
        for block in sub_system.blocks:
            if isinstance(block, SourceBlock):
                if block.port_id in source_port_ids:
                    raise ValueError(
                        f"Duplicate source port id {block.port_id} in sub system {sub_system.name}")
                source_port_ids.add(block.port_id)
            elif isinstance(block, SinkBlock):
                if block.port_id in sink_port_ids:
                    raise ValueError(
                        f"Duplicate sink port id {block.port_id} in sub system {sub_system.name}")
                sink_port_ids.add(block.port_id)

        num_inputs = len(source_port_ids)
        num_outputs = len(sink_port_ids)

        super().__init__(num_inputs=num_inputs, num_outputs=num_outputs, name=name)

        self.sub_system = sub_system
        self.sub_system.compile()

    def update(self, t: float) -> None:
        self.sub_system.update(t)
