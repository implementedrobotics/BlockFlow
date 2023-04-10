from fractions import Fraction
from graphviz import Digraph
from collections import defaultdict, deque
from functools import reduce
from tabulate import tabulate
import time
import math
import numpy as np
from math import gcd

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('QtAgg')


# Utils
def gcd(a, b):
    return math.gcd(a, b)


def lcm(a, b):
    return abs(a * b) // gcd(a, b)


def round_time(t, decimals=6):
    return round(t, decimals)


def gcm_of_floats(float1, float2):
    frac1 = Fraction(float1).limit_denominator()
    frac2 = Fraction(float2).limit_denominator()

    gcd_numerators = gcd(frac1.numerator, frac2.numerator)
    lcm_denominators = lcm(frac1.denominator, frac2.denominator)

    gcm_fraction = Fraction(gcd_numerators, lcm_denominators)
    return float(gcm_fraction)


class Signal:
    def __init__(self, block: "Block", name: str = None) -> None:

        # Signal name
        self.name = name

        # Port ID
        self.port_id = None

        # Block to which the signal belongs
        self.block = block

        # Signal value
        self._data = 0

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, value):
        self._data = value


class Block:
    def __init__(self, num_inputs: int, num_outputs: int, sample_time: float = None, name: str = None) -> None:

        # Block name
        self.name = name

        # Number of inputs and outputs
        self.num_inputs = num_inputs
        self.num_outputs = num_outputs

        # Sample time
        self.sample_time = sample_time
        self.last_sample_time = -1.0

        # List of inputs and output signals

        # Create a list of none values for some number of inputs
        self.inputs = [None] * num_inputs
        self.outputs = [None] * num_outputs

    def _add_signal(self, idx: int, signal: Signal) -> None:

        # Add a signal to the block's input list

        if idx >= self.num_outputs:
            raise ValueError("Invalid output port index")

        # Set the signal's port ID
        signal.port_id = idx

        self.outputs[idx] = signal

    def update(self, t: float) -> None:
        raise NotImplementedError()

    def __str__(self) -> str:
        return f"{self.name} (inputs: {len(self.inputs)}, outputs: {len(self.outputs)})"


class DelayBlock(Block):
    pass


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

    def connect(self, signal: Signal, dest: Block, dest_port: int, dep: bool = True) -> None:

        # Error Check
        all_blocks = self.blocks.copy()
        for block in self.blocks:
            if isinstance(block, SubSystemBlock):
                all_blocks += block.sub_system.blocks

        if signal.block not in all_blocks or dest not in all_blocks:
            raise ValueError("Signal block not in system!")

        if dest_port >= dest.num_inputs:
            raise ValueError("Invalid destination index")

        # Add a dependency from the signal's block to the destination block
        if not isinstance(dest, DelayBlock) and not isinstance(dest, SourceBlock):
            self._add_dependency(signal.block, dest)

        # Connect a signal to a block input
        dest.inputs[dest_port] = signal

        # Update the connections dictionary
        self.connections[(signal.block, signal.port_id)
                         ].append((dest, dest_port))

    def compile(self) -> None:

        self.sorted_blocks = []
        queue = deque()

        # Add blocks with no incoming edges to the queue
        for block in self.blocks:
            if len(self.block_deps[block]) == 0:
                queue.append(block)

        # print("Compile Blocks")
        # print(self.blocks)
        # Process the blocks in the queue
        while queue:
            current_block = queue.popleft()
            self.sorted_blocks.append(current_block)

            # Iterate through all blocks in the system and remove the current block from their dependencies
            for block in self.blocks:
                # print(f"Block: {block.name}")
                # print(f"Block Deps: {self.block_deps[block]}")
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
        for (src_block, _), connected_blocks in self.connections.items():
            for dst_block, _ in connected_blocks:
                graph.edge(str(self.blocks.index(src_block)), str(
                    self.blocks.index(dst_block)))

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

    def print_connections(self) -> None:
        # Prepare the data for the table
        table_data = []

        # Iterate over the connections
        i = 0
        for (src_block, src_port), connected_blocks in self.connections.items():
            for dst_block, dst_port in connected_blocks:
                # Extract information for the table
                from_block = f"{src_block.name}[{src_port}]"
                to_block = f"{dst_block.name}[{dst_port}]"
                description = f"{src_block.name}[{src_port}] --> {dst_block.name}[{dst_port}]"
                data_type = type(src_block.outputs[src_port].data).__name__

                # Add the connection's attributes to the table data
                table_data.append(
                    [i, from_block, to_block, description, data_type])

            i += 1
        # Format the data as a table using the tabulate library
        table_str = tabulate(table_data, headers=[
                             "id", "from", "to", "description", "type"], tablefmt="fancy_grid")

        print(f"Connections:\n{table_str}")

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


# TODO: Port Ids for the source and sink blocks?  This would then map to the system inputs and outputs idx
class SourceBlock(Block):
    def __init__(self, name: str = None):
        super().__init__(num_inputs=1, num_outputs=1, name=name)

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = self.inputs[0].data


class SinkBlock(Block):
    def __init__(self, name: str = None):
        super().__init__(num_inputs=1, num_outputs=1, name=name)

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = self.inputs[0].data


class SubSystemBlock(Block):
    def __init__(self, sub_system: System, name: str = None) -> None:
        num_inputs = 0
        num_outputs = 0

        # Count the number of source and sink blocks in the nested system
        for block in sub_system.blocks:
            if isinstance(block, SourceBlock):
                num_inputs += 1
            elif isinstance(block, SinkBlock):
                num_outputs += 1

        super().__init__(num_inputs=num_inputs, num_outputs=num_outputs, name=name)

        self.sub_system = sub_system
        self.sub_system.compile()

    def update(self, t: float) -> None:
        self.sub_system.update(t)


class Constant(Block):
    def __init__(self, value, sample_time: float = None, name: str = None) -> None:
        super().__init__(num_inputs=0, num_outputs=1, sample_time=sample_time, name=name)
        self.value = value

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t) -> None:
        self.outputs[0].data = (self.value)


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

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        result = 0
        for i, (operation, input) in enumerate(zip(self.operations, self.inputs)):
            if operation == "+":
                result += input.data
            elif operation == "-":
                result -= input.data
            else:
                raise ValueError(
                    f"Invalid operation: {operation} at input index {i}")

        # TODO: Could be faster?
        # result = reduce(lambda acc, item: acc + (item[1].data if item[0] == "+" else -item[1].data),
        #                            zip(self.operations, self.inputs), 0)

        self.outputs[0].data = [result]

        # self.outputs[0].data = (self.inputs[0].data + self.inputs[1].data)


class Div(Block):
    def __init__(self, num_inputs: int = 2, name: str = None) -> None:
        if num_inputs < 2:
            raise ValueError("[Mul]: num_inputs must be at least 2")

        super().__init__(num_inputs=num_inputs, num_outputs=1, name=name)

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t):

        output_data = reduce(
            lambda x, y: x / y, (input_signal.data for input_signal in self.inputs))

        output_data = self.inputs[0].data
        for i in range(1, self.num_inputs):
            output_data /= self.inputs[i].data

        self.outputs[0].data = output_data


class Mul(Block):
    def __init__(self, num_inputs=2, sample_time: float = None, name: str = None) -> None:
        if num_inputs < 2:
            raise ValueError("[Mul]: num_inputs must be at least 2")

        super().__init__(num_inputs=num_inputs, num_outputs=1,
                         sample_time=sample_time, name=name)

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:

        # TODO: May be faster?  But may not be noticeable with low number of inputs
        # output_data = reduce(
        #     lambda x, y: x * y, (input_signal.data for input_signal in self.inputs))

        output_data = self.inputs[0].data
        for i in range(1, self.num_inputs):
            output_data *= self.inputs[i].data

        self.outputs[0].data = output_data


class Gain(Block):
    def __init__(self, gain: float, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.gain = gain

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = (self.inputs[0].data * self.gain)


class Step(Block):
    def __init__(self, T: float = 0, init_val: float = 0, step_val: float = 1, name: str = None) -> None:
        super().__init__(num_inputs=0, num_outputs=1, name=name)
        self.T = T
        self.init_val = init_val
        self.step_val = step_val

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = (self.step_val if t >=
                                self.T else self.init_val)


class ZeroOrderHold(DelayBlock):
    def __init__(self, sample_time, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.sample_time = sample_time
        self.last_sample_time = None

        # Create a signal for the output
        self._add_signal(0, Signal(self))

        # TODO: Initial Value Param?
        self.outputs[0].data = 0

    def update(self, t: float) -> None:
        if self.last_sample_time is None or t >= self.last_sample_time + self.sample_time:
            self.outputs[0].data = self.inputs[0].data
            self.last_sample_time = t


class Integrator(Block):
    def __init__(self, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.prev_val = 0

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = (self.prev_val + self.inputs[0].data)
        self.prev_val = self.outputs[0].data


class Derivative(Block):
    def __init__(self, dt, name: str = None) -> None:
        super().__init__(1, 1)
        self.dt = dt
        self.prev_input = None

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        if self.prev_input is not None:
            self.outputs[0].data = (
                (self.inputs[0].data - self.prev_input) / self.dt)
        self.prev_input = self.inputs[0].data


class Saturation(Block):
    def __init__(self, min_val, max_val, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, name=name)
        self.min_val = min_val
        self.max_val = max_val

        # Create a signal for the output
        self._add_signal(0, Signal(self))

    def update(self, t: float) -> None:
        self.outputs[0].data = (
            min(max(self.inputs[0].data, self.min_val), self.max_val))


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
        print(f"self.time_data: {self.time_data}")
        print(f"self.input_data: {self.input_data}")


constant_5 = Constant(5, sample_time=1, name="Constant 5")
constant_2 = Constant(2, sample_time=1, name="Constant 2")
add_block = Add(name="Add")
add_block_2 = Add(name="Add 2")
zoh = ZeroOrderHold(name="ZOH", sample_time=1)
scope = Scope(num_inputs=1, max_time_steps=100, name="Scope")
add_test = Add(num_inputs=2, name="Sub")
mul_test = Mul(num_inputs=2, sample_time=1, name="Mul")
system = System("Test")

# system.add_block(constant_5)
# system.add_block(constant_2)
# system.add_block(mul_test)
# # system.add_block(add_block)
# # system.add_block(add_block_2)
# # system.add_block(scope)
# # system.add_block(zoh)


# system.connect(constant_5.outputs[0], mul_test, 0)
# system.connect(constant_2.outputs[0], mul_test, 1)
# system.connect(constant_2.outputs[0], mul_test, 2)
# system.connect(constant_2.outputs[0], mul_test, 3)
# system.connect(constant_2.outputs[0], mul_test, 4)
# system.connect(constant_2.outputs[0], mul_test, 5)
# system.connect(constant_2.outputs[0], mul_test, 6)

# system.connect(constant_5.outputs[0], add_block, 0)

# system.connect(zoh.outputs[0], add_block, 0)
# system.connect(constant_2.outputs[0], add_block, 1)
# system.connect(add_block.outputs[0], add_block_2, 0)
# system.connect(constant_2.outputs[0], add_block_2, 1)

# system.connect(add_block_2.outputs[0], scope, 0)
# system.connect(add_block_2.outputs[0], zoh, 0)
# system.compile()


sub_system = System(name="Subsystem")
source = SourceBlock(name="Source")
sink = SinkBlock(name="Sink")
sub_system.add_block(source)
sub_system.add_block(constant_2)
sub_system.add_block(mul_test)
sub_system.add_block(sink)
sub_system.connect(source.outputs[0], mul_test, 0)
sub_system.connect(constant_2.outputs[0], mul_test, 1)
sub_system.connect(mul_test.outputs[0], sink, 0)
sub_system_block = SubSystemBlock(sub_system, "Subsytem Block")


system.add_block(sub_system_block)
system.add_block(constant_5)
system.connect(constant_5.outputs[0], source, 0)
system.compile()
system.update(1)
# print(sub_system)
# sub_system.update(1)

print(f"result: {sink.outputs[0].data}")
print(f"result: {mul_test.outputs[0].data}")


# sub_system.run(3, dt=None)
# print(system)
# system.print()
# system.print_connections()


# Start the timer
# start_time = time.perf_counter()

# Call the function you want to time
# system.update(0)

# Stop the timer
# end_time = time.perf_counter()

# Calculate the elapsed time
# elapsed_time = end_time - start_time

# print(f"Elapsed time: {elapsed_time} seconds")


# scope.view()

# print(system)
# print(f"result: {mul_test.outputs[0].data}")


# Create a Graphviz graph from the system
# graph = system.to_graphviz()


# print(graph)

# Render the graph to a file (e.g., in PNG format)
# graph.render(system.name, format="png")


# # Start the timer
# start_time = time.perf_counter()

# # Call the function you want to time
# my_function()

# # Stop the timer
# end_time = time.perf_counter()

# # Calculate the elapsed time
# elapsed_time = end_time - start_time

# print(f"Elapsed time: {elapsed_time} seconds")
