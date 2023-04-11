from block_flow.blocks.subsystem.sink import SinkBlock
from block_flow.blocks.subsystem.source import SourceBlock
from block_flow.blocks.subsystem.subsystem import SubSystemBlock
from block_flow.blocks.sinks.scope import Scope
from block_flow.blocks.discrete.delay import ZeroOrderHold
from block_flow.systems.system import System
from block_flow.connections.signal import Signal
from block_flow.blocks.sources.constant import Constant
from block_flow.blocks.math import Add, Mul


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
source = SourceBlock(system_parent=sub_system, port_id=0, name="Source")
sink = SinkBlock(system_parent=sub_system, port_id=0, name="Sink")
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

system.run(3, dt=None)

system.print_connections()
sub_system.print_connections()
system.update(1)


# print(f"result: {sink.outputs[0].data}")
# print(f"result: {mul_test.outputs[0].data}")

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
