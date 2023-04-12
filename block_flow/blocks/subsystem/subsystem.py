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

        sources = set()
        sinks = set()

        # Count the number of source and sink blocks in the nested system
        for block in sub_system.blocks:
            if isinstance(block, SourceBlock):
                # if block.port_id in source_ports:
                #     raise ValueError(
                #         f"Duplicate source port id {block.port_id} in sub system {sub_system.name}")
                block.system_parent = self
                sources.add(block)

            elif isinstance(block, SinkBlock):
                # if block.port_id in sink_port_ids:
                #     raise ValueError(
                #         f"Duplicate sink port id {block.port_id} in sub system {sub_system.name}")
                block.system_parent = self
                sinks.add(block)

        num_inputs = len(sources)
        num_outputs = len(sinks)

        super().__init__(num_inputs=num_inputs, num_outputs=num_outputs, name=name)

        source_ports = set()
        sink_ports = set()

        for source in sources:
            # Map the source to the SubSystemBlock input port
            if source.port_id in source_ports:
                raise ValueError(
                    f"Duplicate source port id {source.port_id} in sub system {sub_system.name}")

            self._add_input_port(source.port_id, source.inputs[0])
            source_ports.add(source.port_id)

        for sink in sinks:
            # Map the sink to the SubSystemBlock output port
            if sink.port_id in sink_ports:
                raise ValueError(
                    f"Duplicate sink port id {sink.port_id} in sub system {sub_system.name}")

            self._add_output_port(sink.port_id, sink.outputs[0])
            sink_ports.add(sink.port_id)

        # Compile
        self.sub_system = sub_system
        self.sub_system.compile()

    def update(self, t: float) -> None:
        self.sub_system.update(t)
