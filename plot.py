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

error_positions_abs = [abs(a-b) for (a, b) in zip(z3_positions, pb_positions)]
error_velocities_abs = [abs(a-b) for (a, b) in zip(z3_velocity, pb_velocity)]

print("average velocity error: {}".format(sum(error_velocities_abs)/len(error_velocities_abs)))
print("average position error: {}".format(sum(error_positions_abs)/len(error_positions_abs)))


plt.figure(0)
plt.plot(z3_ts, z3_mdp, label="motor damping proxy")
plt.xlabel("time steps")
plt.ylabel("motor damping proxy")
plt.title("Motor Damping Proxy")
plt.legend()

plt.figure(1)
z3_line = plt.plot(z3_ts, error_positions, label="positions")
pb_line = plt.plot(pb_ts, error_velocities, label="velocities")
plt.setp(z3_line, linewidth=1, color='r')
plt.setp(pb_line, linewidth=1, color='b')
plt.xlabel("time steps")
plt.ylabel("error")
plt.title('Position and Velocity Errors')
plt.legend()

plt.figure(2)
z3_line = plt.plot(z3_ts, z3_positions, label="z3")
pb_line = plt.plot(pb_ts, pb_positions, label="pb")
plt.setp([z3_line], linestyle='-.')
plt.setp([pb_line], linestyle=':')
plt.setp(z3_line, linewidth=1, color='r')
plt.setp(pb_line, linewidth=1, color='b')
plt.xlabel("time steps")
plt.ylabel("position (radians)")
plt.title('Positions')
plt.legend()

plt.figure(3)
z3_line = plt.plot(z3_ts, z3_velocity, label="z3")
pb_line = plt.plot(pb_ts, pb_velocity, label="pb")
plt.setp([z3_line], linestyle='-.')
plt.setp([pb_line], linestyle=':')
plt.setp(z3_line, linewidth=1, color='r')
plt.setp(pb_line, linewidth=1, color='b')
plt.title('Velocity')
plt.xlabel("time steps")
plt.ylabel("velocity (radians/sec)")
plt.legend()

plt.figure(4)
z3_line = plt.plot(z3_ts, z3_positions, label="z3")
pb_line = plt.plot(pb_ts, pb_positions, label="pb")
plt.setp([z3_line], linestyle='-.')
plt.setp([pb_line], linestyle=':')
plt.setp(z3_line, linewidth=1, color='r')
plt.setp(pb_line, linewidth=1, color='b')
plt.xlabel("time steps")
z3_line = plt.plot(z3_ts, z3_velocity, label="z3")
pb_line = plt.plot(pb_ts, pb_velocity, label="pb")
plt.setp([z3_line], linestyle=':')
plt.setp([pb_line], linestyle='--')
plt.setp(z3_line, linewidth=1)#, color='r')
plt.setp(pb_line, linewidth=1)#, color='b')
plt.legend()

plt.show()
