import json
import matplotlib.pyplot as plt

f_pb = open("position_velocity_pybullet_data.txt")
f_z3 = open("position_velocity_z3_data.txt")

pb_blob = json.load(f_pb)
z3_blob = json.load(f_z3)

z3_ts = [int(ts) for ts in z3_blob]
z3_positions = [z3_blob[key]["position"] for key in z3_blob]
z3_velocity = [z3_blob[key]["velocity"] for key in z3_blob]
z3_mdp = [z3_blob[key]["mdp"] for key in z3_blob]

pb_ts = [int(ts) for ts in pb_blob]
pb_positions = [pb_blob[key]["position"] for key in pb_blob]
pb_velocity = [pb_blob[key]["velocity"] for key in pb_blob]

error_positions = [(a-b) for (a, b) in zip(z3_positions, pb_positions)]
error_velocities = [(a-b) for (a, b) in zip(z3_velocity, pb_velocity)]

plt.figure(0)
plt.plot(z3_ts, z3_mdp, label="motor damping proxy")
plt.title("Motor Damping Proxy")
plt.legend()

plt.figure(1)
plt.plot(z3_ts, error_positions, label="positions")
plt.plot(pb_ts, error_velocities, label="velocities")
plt.title('Error')
plt.legend()

plt.figure(2)
plt.plot(z3_ts, z3_positions, label="z3")
plt.plot(pb_ts, pb_positions, label="pb")
plt.title('Positions')
plt.legend()

plt.figure(3)
plt.plot(z3_ts, z3_velocity, label="z3")
plt.plot(pb_ts, pb_velocity, label="pb")
plt.title('Velocity')
plt.legend()
plt.show()
