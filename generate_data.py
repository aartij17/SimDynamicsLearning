import math

import numpy as np
import matplotlib.pyplot as plt
from pendulum_z3_synthesize_mdp import solve_mdp_analysis, init_synthesize
from pendulum_manipulator import simulate_pendulums

"""
-------> POSITIONS
|       [mdp1, mdp2, 0, 0, 0]
|       [0, 0, 0, 0, 0]
|       [0, 0, 0, 0, 0]
|       [0, 0, 0, 0, 0]
|       [0, 0, 0, 0, 0]
|       [0, 0, 0, 0, 0]
|       [0, 0, 0, 0, 0]
V       [0, 0, 0, 0, 0]
"""
PI = math.pi
initial_positions = [1, PI/2, PI, 1.5 * PI, 2 * PI]
initial_velocities = [0.5 * PI, PI, 1.5 * PI, 2 * PI, 2.5 * PI]#, 3 * PI, 3.5 * PI, 4 * PI]

data_matrix = [[None]*len(initial_positions) for _ in range(len(initial_velocities))]

print(data_matrix)

for vel_index, vel in enumerate(initial_velocities):
    for pos_index, pos in enumerate(initial_positions):
        simulate_pendulums(pos, vel)
        init_synthesize()
        ret_val = solve_mdp_analysis(pos, vel)
        #print("vel_index:: {}, pos_index: {}, RET VAL: {}".format(vel_index, pos_index, ret_val))
        data_matrix[vel_index][pos_index] = ret_val


print(data_matrix)
H = np.array(data_matrix)
plt.imshow(H)
plt.legend()
plt.show()
