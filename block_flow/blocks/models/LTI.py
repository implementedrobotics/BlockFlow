

from block_flow.connections.port import OutputPort, InputPort
from block_flow.blocks.block import Block

import numpy as np

# def discretize_bilinear(A,B,C,D,T):
#   # Compute Ad using matrix inversion
#   Ad = np.linalg.inv(np.eye(A.shape[0]) - 0.5*A*T) @ (np.eye(A.shape[0]) + 0.5*A*T)

#   # Compute Bd using matrix multiplication
#   Bd = np.linalg.inv(np.eye(A.shape[0]) - 0.5*A*T) @ (B*T)

#   # Cd and Dd are unchanged
#   Cd = C
#   Dd = D

#   return Ad,Bd,Cd,Dd


# # Function that takes continuous-time matrices A,B,C,D and sampling interval T
# # Returns discrete-time matrices Ad,Bd,Cd,Dd using zero order hold approximation
# def discretize_zoh(A,B,C,D,T):
#   # Compute e^(A*T) using numpy's exp() function
#     Ad = scipy.linalg.expm(A*T)
#     print(Ad)
#     Ad = np.exp(A*T)
#     print(Ad)


#     # Compute Bd using matrix multiplication
#     Bd = np.dot(Ad,np.dot(B,T))

#     # Cd and Dd are unchanged
#     Cd = C
#     Dd = D


# Linear time invariant system
# TODO: Discretize function
class LTI(Block):
    def __init__(self, A: np.ndarray, B: np.ndarray, C: np.ndarray | None, D: np.ndarray | None, x_0: np.ndarray | None, sample_time: float = None, name: str = None) -> None:
        super().__init__(num_inputs=1, num_outputs=1, sample_time=sample_time, name=name)

        # Check that the dimensions are correct
        if A.shape[0] != A.shape[1]:
            raise ValueError("A must be a square matrix")

        self.num_states = A.shape[0]

        if B.shape[0] != A.shape[0]:
            raise ValueError("B must have the same number of rows as A")

        self.num_inputs = B.shape[1]

        if C is not None:
            if C.shape[1] != A.shape[0]:
                raise ValueError("C must have the same number of columns as A")
            self.num_outputs = C.shape[0]
        else:
            self.num_outputs = self.num_states  # Full State Output
            self.C = np.eye(self.num_states)

        if D is not None:
            if D.shape[0] != C.shape[0]:
                raise ValueError("D must have the same number of rows as C")
            if D.shape[1] != B.shape[1]:
                raise ValueError("D must have the same number of columns as B")

            self.D = D
        else:
            self.D = np.zeros((self.num_outputs, B.shape[1]))

        # Store the system matrices
        self.A = A
        self.B = B

        # Update the initial state vector
        if x_0 is None:
            self._x_0 = np.zeros((self.num_states, 1))
        else:
            self._x_0 = x_0

        # Create the state vector
        self._x = self._x_0

        # Create a port for the output
        self._add_output_port(0, OutputPort(
            self, data_types=(np.ndarray), name="Y"))

        # Create a ports for the inputs
        self._add_input_port(0, InputPort(
            self, data_types=(np.ndarray)))

        # Set Initial Output
        self.outputs[0].data = (self._x)

    def update(self, t) -> None:

        # Update the state vector
        self._x = self.A @ self._x + self.B @ self.inputs[0].data
        y_k = self.C @ self._x + self.D @ self.inputs[0].data

        # Set Output
        self.outputs[0].data = y_k
