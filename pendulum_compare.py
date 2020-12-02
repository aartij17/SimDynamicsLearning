import argparse
import json
import time

# state the number of links in your urdf (edit this as necessary)
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='example script to demonstrate pybullet simulator and control of a 2D N-link robot')  # add capability of user input
    parser.add_argument('-num_links', '--number_of_links_urdf', help='number of links in urdf', type=int,
                        default=1)  # set default input to 1 if none is given
    arguments = parser.parse_args()
    print('type: ', type(arguments.number_of_links_urdf))
    number_of_links_urdf = int(arguments.number_of_links_urdf)  # define the number of links

# I ran this script using python 3.7.2
# import the os module (needed when p.loadURDF() method is invoked)
import os
# imports the pybullet simulator API
import pybullet as p
# imports the pybullet_data module (used to figure out where the robot model needs to be loaded from)
import pybullet_data

# allows python script to be called with inputs

# loads the pybullet physics simulator (with no robot yet)
p.connect(p.GUI)

# this prints out where the dataPath base directory is. The DataPath base directory is the default directory where pybullet looks in order to load a robot model (when the p.loadURDF() method is invoked)
print(pybullet_data.getDataPath())

# state the directory extension for pybullet to find your urdf (edit this as necessary)
# model_urdf_directory_extension = "romans_urdf_files/octopus_files/python_scripts_edit_urdf/octopus_generated_"+str(number_of_links_urdf)+"_links.urdf"

# if you do not want to create an extension simply uncomment this line
model_urdf_directory_extension = "cs292c_robot_models/octopus_generated_" + str(number_of_links_urdf) + "_links.urdf"

# this line loads a URDF (3D robot model) into pybullet physics simulator, and it returns a unique ID. That unique ID is needed when querying information about the robot. It is also used when controlling the robot
# octopusBodyUniqueId = p.loadURDF( fileName=os.path.join(pybullet_data.getDataPath(), model_urdf_directory_extension), flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

# print(os.getcwd())
# p.setAdditionalSearchPath(os.getcwd())
pendulum_uniqueId_pybullet = p.loadURDF(
    fileName=os.path.join(pybullet_data.getDataPath(), model_urdf_directory_extension),
    flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
pendulum_uniqueId_z3 = p.loadURDF(fileName=os.path.join(pybullet_data.getDataPath(), model_urdf_directory_extension),
                                  flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

p.setCollisionFilterPair(bodyUniqueIdA=pendulum_uniqueId_pybullet,
                         bodyUniqueIdB=pendulum_uniqueId_z3,
                         linkIndexA=0,
                         linkIndexB=0,
                         enableCollision=0
                         )

p.setCollisionFilterPair(bodyUniqueIdA=pendulum_uniqueId_pybullet,
                         bodyUniqueIdB=pendulum_uniqueId_z3,
                         linkIndexA=0,
                         linkIndexB=-1,
                         enableCollision=0
                         )
p.setCollisionFilterPair(bodyUniqueIdA=pendulum_uniqueId_pybullet,
                         bodyUniqueIdB=pendulum_uniqueId_z3,
                         linkIndexA=-1,
                         linkIndexB=0,
                         enableCollision=0
                         )
# enables gravity
p.setGravity(0, 0, -9.8)

# sets real time simulation
p.setRealTimeSimulation(
    enableRealTimeSimulation=1)  # now we dont have to call p.stepSimulation() in order to advance the time step of the simulation environment

# turn off all motors so that joints are not stiffened for the rest of the simulations
p.setJointMotorControlArray(
    bodyUniqueId=pendulum_uniqueId_pybullet,
    jointIndices=list(range(number_of_links_urdf)),
    controlMode=p.TORQUE_CONTROL,
    positionGains=[0.1] * number_of_links_urdf,
    velocityGains=[0.1] * number_of_links_urdf,
    forces=[0] * number_of_links_urdf
)

# load the block thatwe are trying to avoid


targetPositions = [0] * (number_of_links_urdf)
targetPositions[0] = 3.14 * (36 / 360)  # set desired joint angles in radians
targetVelocities = [0] * (number_of_links_urdf)  # set desired joint velocities

# move the robot according to desited joint angles and desired joint velocities using PD control
p.setJointMotorControlArray(
    bodyUniqueId=pendulum_uniqueId_pybullet,
    jointIndices=list(range(number_of_links_urdf)),
    controlMode=p.POSITION_CONTROL,
    # targetPositions=targetPositions,
    velocityGains=[0] * number_of_links_urdf,
    positionGains=[0] * number_of_links_urdf,
    forces=[0] * number_of_links_urdf
)

p.setJointMotorControlArray(
    bodyUniqueId=pendulum_uniqueId_z3,
    jointIndices=list(range(number_of_links_urdf)),
    controlMode=p.POSITION_CONTROL,
    # targetPositions=targetPositions,
    velocityGains=[0] * number_of_links_urdf,
    positionGains=[0] * number_of_links_urdf,
    forces=[0] * number_of_links_urdf
)
p.changeDynamics(
    bodyUniqueId=pendulum_uniqueId_pybullet,
    linkIndex=0,
    jointDamping=0.1
)

p.changeDynamics(
    bodyUniqueId=pendulum_uniqueId_z3,
    linkIndex=0,
    jointDamping=0.1
)

j = 0
p.resetJointState(
    bodyUniqueId=pendulum_uniqueId_pybullet,
    jointIndex=0,
    targetValue=1,
    targetVelocity=0
)

p.resetJointState(
    bodyUniqueId=pendulum_uniqueId_z3,
    jointIndex=0,
    targetValue=1,
    targetVelocity=0
)

f_z3 = open("position_velocity_z3_data.txt")
lines = f_z3.read()
f_z3.close()
z3_data_blob = json.loads(lines)
z3_sorted_ts = sorted(map(lambda x: int(x), z3_data_blob.keys()))

f_pybullet = open("position_velocity_pybullet_data.txt")
lines = f_pybullet.read()
f_pybullet.close()
pybullet_data_blob = json.loads(lines)
pybullet_sorted_ts = sorted(map(lambda x: int(x), pybullet_data_blob.keys()))

timesteps = json.load(open("config.json"))["time_steps"]
i = 0
while i < timesteps:
    if i not in z3_sorted_ts:
        i += 1
        continue
    p.resetJointState(
        bodyUniqueId=pendulum_uniqueId_pybullet,
        jointIndex=0,
        targetValue=z3_data_blob[str(i)]["position"],
        targetVelocity=z3_data_blob[str(i)]["velocity"]
    )

    p.resetJointState(
        bodyUniqueId=pendulum_uniqueId_z3,
        jointIndex=0,
        targetValue=pybullet_data_blob[str(i)]["position"],
        targetVelocity=pybullet_data_blob[str(i)]["velocity"]
    )
    a = p.getJointStates(bodyUniqueId=pendulum_uniqueId_pybullet, jointIndices=list(range(number_of_links_urdf)))
    b = p.getJointStates(bodyUniqueId=pendulum_uniqueId_z3, jointIndices=list(range(number_of_links_urdf)))

    i += 1
