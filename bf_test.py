from block_flow.blocks.subsystem.sink import SinkBlock
from block_flow.blocks.subsystem.source import SourceBlock
from block_flow.blocks.subsystem.subsystem import SubSystemBlock
from block_flow.blocks.sinks.scope import Scope
from block_flow.blocks.sinks.MujocoVisualizer import MuJoCoVisualizationBlock
from block_flow.blocks.discrete.delay import ZeroOrderHold
from block_flow.systems.system import System
from block_flow.connections.signal import Signal
from block_flow.blocks.sources.constant import Constant
from block_flow.blocks.math import Add, Mul
from block_flow.blocks.models.LTI import LTI
from block_flow.blocks.models.MujocoPlant import MujocoPlant

from scipy.signal import cont2discrete

import numpy as np
import time
# zoh = ZeroOrderHold(name="ZOH", sample_time=1)

# Parameters
m1 = 1
m2 = 2
k1 = 10
k2 = 15
c1 = 0.5
c2 = 1

A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [-k1/m1, k1/m1, -c1/m1, c1/m1],
              [k1/m2, -k2/m2, c1/m2, -(c1+c2)/m2]])

B = np.array([0, 0, 1/m1, 0]).reshape(4, 1)

C = np.eye(4)  # States
D = np.zeros((4, 1))  # States x Inputs


Ad, Bd, _, _, _ = cont2discrete((A, B, C, D), dt=0.01, method='bilinear')


# Initial conditions and external force
x0 = np.array([0, 0, 0, 0])
u = 2  # No external force applied


system = System("Test")


ss = system.add_block(LTI(A=Ad, B=Bd, C=None, D=None, x_0=x0, name="M-S-D"))
u_ref = system.add_block(Constant(np.array([u]), name="Input 1"))
scope = system.add_block(
    Scope(num_inputs=1, max_time_steps=5000, name="Scope"))
# viz = system.add_block(MuJoCoVisualizationBlock(
#    model_path="cart.xml", sample_time=0.01, name="Mujoco Viz"))

# time.sleep(1)
cart = system.add_block(MujocoPlant(
    model_path="cart.xml", x_0=x0, sample_time=0.01, name="Cart Pendulum"))

# Connect the blocks
# system.connect(u_ref.outputs[0], ss.inputs[0])
system.connect(u_ref.outputs[0], cart.inputs[0])
# system.connect(cart.outputs[0], viz.inputs[0])

# sub_system = System(name="Subsystem")


# # Add Subsystem Blocks
# constant_2 = sub_system.add_block(
#     Constant(2.1, sample_time=1, name="Constant 2"))
# mul_block = sub_system.add_block(Mul(num_inputs=2, sample_time=1, name="Mul"))

# # Add Source and Sinks
# source = sub_system.add_block(SourceBlock(port_id=0, name="Source"))
# sink = sub_system.add_block(SinkBlock(port_id=0, name="Sink"))

# # Connect the blocks
# sub_system.connect(source.outputs[0], mul_block.inputs[0])
# sub_system.connect(constant_2.outputs[0], mul_block.inputs[1])
# sub_system.connect(mul_block.outputs[0], sink.inputs[0])

# # Create Subsystem Block
# sub_system_block = system.add_block(SubSystemBlock(sub_system, "Subsystem"))

# constant_5 = system.add_block(Constant(5, sample_time=1, name="Constant 5"))


# # Connect the blocks
# system.connect(sub_system_block.outputs[0], scope.inputs[0])
# system.connect(constant_5.outputs[0], sub_system_block.inputs[0])

# Compile the system
system.compile()

# Debug Print Connections
system.print_connections()

# Run the system
system.run(5, dt=0.01)

system.stop()
# Hold Plot
# scope.view()

# print(f"System: {ss.outputs[0].data}")

# print(sub_system_block.outputs[0].data)
# system.print_connections()


# print(system.to_graphviz())

# system.to_graphviz().render(system.name, format="png")
# sub_system.print_connections()
# system.update(1)

# Start the timer
# start_time = time.perf_counter()

# Call the function you want to time
# system.update(0)

# Stop the timer
# end_time = time.perf_counter()

# Calculate the elapsed time
# elapsed_time = end_time - start_time

# print(f"Elapsed time: {elapsed_time} seconds")
