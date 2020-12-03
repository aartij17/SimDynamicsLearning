import argparse
import json
import math
import time

number_of_links_urdf = 1
# state the number of links in your urdf (edit this as necessary)
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser(
#         description='example script to demonstrate pybullet simulator and control of a 2D N-link robot')  # add capability of user input
#     parser.add_argument('-num_links', '--number_of_links_urdf', help='number of links in urdf', type=int,
#                         default=1)  # set default input to 1 if none is given
#     arguments = parser.parse_args()
#     print('type: ', type(arguments.number_of_links_urdf))
#     number_of_links_urdf = int(arguments.number_of_links_urdf)  # define the number of links

# I ran this scrip[t using python 3.7.2
# import the os mudule (needed when p.loadURDF() method is invoked)
import os
# imports the pybullet simulator API
import pybullet as p
# imports the pybullet_data module (used to figure out where the robot model needs to be loaded from)
import pybullet_data
# allows python script to be called with inputs
import argparse

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
    enableRealTimeSimulation=0)  # now we dont have to call p.stepSimulation() in order to advance the timestep of the simulation environment

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

initial_position = 15
initial_velocity = 1

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
    targetValue=initial_position,
    targetVelocity=initial_velocity
)

p.resetJointState(
    bodyUniqueId=pendulum_uniqueId_z3,
    jointIndex=0,
    targetValue=initial_position,
    targetVelocity=initial_velocity
)

time_step = 0

previous_theta = []
previous_theta_d = 0
# check equality
data = {}

config_file = open("config.json")
config = json.load(config_file)

"""
take the values from Z3, and call resetJointState, 
{"0": {"position": 1.0, "velocity": 0.0}, "23": {"position": 1.0141867807385068, 
"velocity": 0.40087002089676194}, "196": {"position": 1.015961884242391, 
"velocity": 0.4260248409322054}, "247": {"position": 1.0587030949993685, 
"velocity": 0.8321196745638638}, "431":
"""


def simulate_pendulums(init_position=None, init_velocity=None):
    global initial_velocity, initial_position, number_of_links_urdf, data, p
    if init_velocity and init_velocity:
        initial_velocity = init_velocity
        initial_position = init_position

    p.resetJointState(
        bodyUniqueId=pendulum_uniqueId_pybullet,
        jointIndex=0,
        targetValue=initial_position,
        targetVelocity=initial_velocity
    )

    p.resetJointState(
        bodyUniqueId=pendulum_uniqueId_z3,
        jointIndex=0,
        targetValue=initial_position,
        targetVelocity=initial_velocity
    )
    f = open("position_velocity_pybullet_data.txt", "w")

    i = 0
    while i < config["time_steps"]:
        a = p.getJointStates(bodyUniqueId=pendulum_uniqueId_pybullet, jointIndices=list(range(number_of_links_urdf)))
        b = p.getJointStates(bodyUniqueId=pendulum_uniqueId_z3, jointIndices=list(range(number_of_links_urdf)))
        data[i] = {
            "position": a[0][0],
            "velocity": a[0][1]
        }
        i += 1
        p.stepSimulation()
        #time.sleep(1)


    f.write(json.dumps(data))
    f.close()


if __name__ == "__main__":
    simulate_pendulums()
