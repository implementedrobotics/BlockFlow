# BlockFlow - A Modular Block-Based Simulation Framework

BlockFlow is a modular, block-based simulation framework designed for creating and running simple to complex systems, incorporating different components with various input/output relationships. The framework is written in Python and supports custom block creation and organization, and includes support for single float data type.

## Features

- Simple and intuitive block-based system design
- Customizable block creation for diverse components
- Support for single float data type
- Designed for easy integration and extensibility
- Built-in support for common mathematical operations
- Topological sorting to resolve system dependencies
- Visual representation of system using Graphviz
- Utility functions for system management, debugging, and visualization

## Getting Started

To start using BlockFlow, clone this repository or download the source code. Install any required dependencies and start creating your own systems.

### Installation

```
pip install -r requirements.txt
```

### Usage

1. Import the necessary components from the blockflow module.
2. Create a new System instance.
3. Define your custom Block classes, if needed.
4. Instantiate your Block objects and add them to the System.
5. Connect the input and output signals between blocks.
6. Compile and run the system.

## Example

```
from blockflow import System, SourceBlock, SinkBlock, ConstantBlock, MulBlock

# Create a new system
system = System(name="ExampleSystem")

# Create blocks
source = SourceBlock(system_parent=system, port_id=0, name="Source")
constant_2 = ConstantBlock(system_parent=system, value=2, name="Constant_2")
mul_test = MulBlock(system_parent=system, name="Mul_Test")
sink = SinkBlock(system_parent=system, port_id=0, name="Sink")

# Add blocks to the system
system.add_block(source)
system.add_block(constant_2)
system.add_block(mul_test)
system.add_block(sink)

# Connect the blocks
system.connect(source.outputs[0], mul_test, 0)
system.connect(constant_2.outputs[0], mul_test, 1)
system.connect(mul_test.outputs[0], sink, 0)

# Compile and run the system
system.compile()
system.run(3, dt=None)
```

## Documentation

Detailed documentation for the BlockFlow framework can be found in the source code.

## Contributing

Contributions to the BlockFlow project are welcome. Please open an issue or submit a pull request with your proposed changes.

## License

BlockFlow is released under the [MIT License](https://opensource.org/license/mit/).
