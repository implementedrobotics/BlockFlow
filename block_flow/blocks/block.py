from block_flow.connections.signal import Signal


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
