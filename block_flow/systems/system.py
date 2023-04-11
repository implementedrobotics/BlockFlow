from tabulate import tabulate
from collections import defaultdict, deque
from block_flow.connections.port import OutputPort, InputPort
from block_flow.blocks.subsystem.source import SourceBlock
from block_flow.blocks.block import Block
from block_flow.blocks.subsystem.subsystem import SubSystemBlock
from block_flow.connections.signal import Signal
from block_flow.blocks.discrete.delay import DelayBlock
from block_flow.utils.time import lcm, gcm_of_floats, round_time

from graphviz import Digraph
import time


class System:
    def __init__(self, name: str = None) -> None:

        # System name
        self.name = name

        # List of blocks in the system
        self.blocks = []

        # List of blocks sorted in topological order
        self.sorted_blocks = []

        # List of block dependencies
        self.block_deps = defaultdict(set)

        # List of connections between blocks
        self.connections = defaultdict(list)

        # Flag to indicate if the system has been compiled
        self.compiled = False

    def add_block(self, block: Block) -> None:

        # Add a block to the system
        self.blocks.append(block)

        # Invalidate block
        self.compiled = False

    def _add_dependency(self, source: Block, dest: Block) -> None:

        # Invalidate block
        self.compiled = False

        # Add a dependency from source to dest block
        self.block_deps[source].add(dest)

    def lcm_sample_time(self) -> int:
        lcm_value = 1
        for block in self.blocks:
            if block.sample_time is not None:
                lcm_value = lcm(lcm_value, block.sample_time)
        return lcm_value

    def gcd_sample_time(self) -> float:
        gcd_value = None
        for block in self.blocks:
            if block.sample_time is not None:
                if gcd_value is None:
                    gcd_value = block.sample_time
                else:
                    gcd_value = gcm_of_floats(gcd_value, block.sample_time)
        return gcd_value

    def run(self, duration: float | None = None, dt: float | None = None) -> None:

        if dt is None:
            dt = self.gcd_sample_time()

        print(f"Running system for {duration} seconds with dt={dt}...")
        start_time = time.time()
        t = 0
        if duration is None:
            # Run forever
            while True:
                t = round_time(time.time() - start_time)
                self.update(t)
                time.sleep(dt)
        else:
            # Run for the specified duration
            while time.time() - start_time < duration:
                t = round_time(time.time() - start_time)
                self.update(t)
                time.sleep(dt)

    def update(self, t: float) -> None:

        if not self.compiled:
            raise ValueError("Block system has not been compiled!")

        epsilon = 1e-2
        for block in self.sorted_blocks:
            if block.sample_time is None or abs(t % block.sample_time) < epsilon:
                block.update(t)

    # def connect(self, signal: Signal, dest: Block, dest_port: int, dep: bool = True) -> None:

    #     # Error Check
    #     all_blocks = self.blocks.copy()
    #     for block in self.blocks:
    #         if isinstance(block, SubSystemBlock):
    #             all_blocks += block.sub_system.blocks

    #     if signal.block not in all_blocks or dest not in all_blocks:
    #         raise ValueError("Signal block not in system!")

    #     if dest_port >= dest.num_inputs:
    #         raise ValueError("Invalid destination index")

    #     # Add a dependency from the signal's block to the destination block
    #     if not isinstance(dest, DelayBlock) and not isinstance(dest, SourceBlock):
    #         self._add_dependency(signal.block, dest)

    #     # Connect a signal to a block input
    #     dest.inputs[dest_port] = signal

    #     # Do some type mapping if this is a source/sink block from a subsystem.  For better debug print
    #     if isinstance(dest, SourceBlock):
    #         dest_port = dest.port_id
    #         dest = dest.system_parent

    #     # Update the connections dictionary
    #     self.connections[(signal.block, signal.port_id)
    #                      ].append((dest, dest_port))

    def connect(self, source: OutputPort, dest: InputPort, dep: bool = True) -> None:

        # Error Check
        if source is None:
            raise ValueError("Invalid Output Port")

        if dest is None:
            raise ValueError("Invalid Input Port")

        all_blocks = self.blocks.copy()
        for block in self.blocks:
            if isinstance(block, SubSystemBlock):
                all_blocks += block.sub_system.blocks

        if source.block not in all_blocks or dest.block not in all_blocks:
            raise ValueError("Block Port not in system!")

        # Add a dependency from the signal's block to the destination block
        if not isinstance(dest.block, DelayBlock) and not isinstance(dest.block, SourceBlock):
            self._add_dependency(source.block, dest.block)

        # Connect a signal to a block input

        if dest.connected:
            raise ValueError("Input Port already connected")

        # dest._data = source._data
        # dest.inputs[dest_port] = signal

        dest.connected = source._connect(dest)

        # Do some type mapping if this is a source/sink block from a subsystem.  For better debug print
        # if isinstance(dest.block, SourceBlock):
        # dest_port = dest.port_id
        #   dest = dest.block.system_parent

        # Update the connections dictionary
        self.connections[source].append(dest)

    def compile(self) -> None:

        self.sorted_blocks = []
        queue = deque()

        # Add blocks with no incoming edges to the queue
        for block in self.blocks:
            if len(self.block_deps[block]) == 0:
                queue.append(block)

        # Process the blocks in the queue
        while queue:
            current_block = queue.popleft()
            self.sorted_blocks.append(current_block)

            # Iterate through all blocks in the system and remove the current block from their dependencies
            for block in self.blocks:
                if current_block in self.block_deps[block]:
                    self.block_deps[block].remove(current_block)
                    if len(self.block_deps[block]) == 0:
                        queue.append(block)

        # If the number of sorted blocks is not equal to the number of blocks in the system,
        # then there is a cycle in the system
        if len(self.sorted_blocks) != len(self.blocks):
            raise ValueError("The system has circular dependencies")

        # Reverse the sorted blocks list
        self.sorted_blocks.reverse()
        self.compiled = True

    def to_graphviz(self) -> Digraph:
        # Create a new directed graph
        graph = Digraph(self.name)

        # Set graph attributes for left-to-right layout and orthogonal edges
        graph.attr(rankdir="LR", splines="ortho")

        graph.node("title", label=self.name, shape="plaintext")
        graph.attr("node", shape="box")

        # Add nodes (blocks) to the graph
        for idx, block in enumerate(self.blocks):
            graph.node(str(idx), label=block.name, shape="box")

        # Add edges (connections) to the graph
        for src, connected_blocks in self.connections.items():
            for dst in connected_blocks:
                graph.edge(str(self.blocks.index(src.block)), str(
                    self.blocks.index(dst.block)))

         # Position the title node at the top of the graph
        graph.attr(rank="min", rankdir="LR")
        # Add an invisible edge to maintain the desired layout
        graph.edge("title", str(0), style="invis", constraint="false")

        return graph

    def print(self) -> None:
        # Prepare the data for the table
        table_data = []

        # Iterate over the blocks in the system
        for i, block in enumerate(self.blocks):
            block_name = block.name
            num_inputs = block.num_inputs
            num_outputs = block.num_outputs
            block_type = block.__class__.__name__

            # Add the block's attributes to the table data
            table_data.append(
                [i, block_name, num_inputs, num_outputs, block_type])

        # Format the data as a table using the tabulate library
        table_str = tabulate(table_data, headers=[
                             "Id", "Name", "Inputs", "Outputs", "Type"], tablefmt="fancy_grid")

        print(f"System [{self.name}]:\n\n{table_str}")

    # def print_connections(self) -> None:
    #     # Prepare the data for the table
    #     table_data = []

    #     # Iterate over the connections
    #     i = 0
    #     for (src_block, src_port), connected_blocks in self.connections.items():
    #         for dst_block, dst_port in connected_blocks:
    #             # Extract information for the table
    #             from_block = f"{src_block.name}[{src_port}]"
    #             to_block = f"{dst_block.name}[{dst_port}]"
    #             description = f"{src_block.name}[{src_port}] --> {dst_block.name}[{dst_port}]"
    #             data_type = type(src_block.outputs[src_port].data).__name__

    #             # Add the connection's attributes to the table data
    #             table_data.append(
    #                 [i, from_block, to_block, description, data_type])

    #         i += 1
    #     # Format the data as a table using the tabulate library
    #     table_str = tabulate(table_data, headers=[
    #                          "id", "from", "to", "description", "type"], tablefmt="fancy_grid")

    #     print(f"{self.name} Connections:\n{table_str}\n")

    def print_connections(self) -> None:
        # Prepare the data for the table
        table_data = []

        # Iterate over the connections
        i = 0
        for src_port, connected_blocks in self.connections.items():
            for dst_port in connected_blocks:
                dst_block = dst_port.block
                if isinstance(dst_block, SourceBlock):
                    dst_block = dst_block.system_parent

                # Extract information for the table
                from_block = f"{src_port.block.name}[{src_port.port_id}]"
                to_block = f"{dst_block.name}[{dst_port.port_id}]"
                description = f"{src_port.block.name}[{src_port.port_id}] --> {dst_block.name}[{dst_port.port_id}]"
                # data_type = type(
                #     src_port.block.outputs[src_port.port_id].data).__name__
                data_type = src_port.data_type.__name__

                # Add the connection's attributes to the table data
                table_data.append(
                    [i, from_block, to_block, description, data_type])

            i += 1
        # Format the data as a table using the tabulate library
        table_str = tabulate(table_data, headers=[
                             "id", "from", "to", "description", "type"], tablefmt="fancy_grid")

        print(f"{self.name} Connections:\n{table_str}\n")

        # Reset the table data
        table_data = []
        # Print Not Connected Ports in the System
        for block in self.blocks:
            for port in block.inputs:
                if port.connected == False:
                    # Add the connection's attributes to the table data
                    table_data.append([port.block.name, port.port_id])

        if len(table_data) > 0:
            # Format the data as a table using the tabulate library
            table_str = tabulate(table_data, headers=[
                "Block Name", "Port"], tablefmt="fancy_grid")

            print(f"{self.name} Unconnectioned:\n{table_str}\n")

    def __str__(self):

        # String representation for blocks
        blocks_str = "\n".join(str(block) for block in self.blocks)

        # String representation for connections
        connections_str = "\n".join(
            f"{src_block.name}[{src_port}] -> {dst_block.name}[{dst_port}]"
            for (src_block, src_port), connected_blocks in self.connections.items()
            for dst_block, dst_port in connected_blocks
        )

        return f"System:\nBlocks:\n{blocks_str}\n\nConnections:\n{connections_str}"
