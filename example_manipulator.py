import argparse

#state the number of links in your urdf (edit this as necessary)
if __name__ == '__main__':
    parser=argparse.ArgumentParser(description='example script to demonstrate pybullet simulator and control of a 2D N-link robot')  #add capability of user input
    parser.add_argument('-num_links', '--number_of_links_urdf', help='number of links in urdf', type=int, default=1 )  # set default input to 1 if none is given
    arguments=parser.parse_args()
    print('type: ', type(arguments.number_of_links_urdf))
    number_of_links_urdf=int(arguments.number_of_links_urdf) #define the number of links


#I ran this scrip[t using python 3.7.2
#import the os mudule (needed when p.loadURDF() method is invoked)
import os
#imports the pybullet simulator API
import pybullet as p
#imports the pybullet_data module (used to figure out where the robot model needs to be loaded from)
import pybullet_data
#allows python script to be called with inputs
import argparse

#loads the pybullet physics simulator (with no robot yet)
p.connect(p.GUI) 

#this prints out where the dataPath base directory is. The DataPath base directory is the default directory where pybullet looks in order to load a robot model (when the p.loadURDF() method is invoked)
print( pybullet_data.getDataPath() )

#state the directory extension for pybullet to find your urdf (edit this as necessary)
#model_urdf_directory_extension = "romans_urdf_files/octopus_files/python_scripts_edit_urdf/octopus_generated_"+str(number_of_links_urdf)+"_links.urdf"

#if you do not want to create an extension simply uncomment this line
model_urdf_directory_extension = "cs292c_robot_models/octopus_generated_"+str(number_of_links_urdf)+"_links.urdf"


#this line loads a URDF (3D robot model) into pybullet physics simulator, and it returns a unique ID. That unique ID is needed when querying information about the robot. It is also used when controlling the robot
#octopusBodyUniqueId = p.loadURDF( fileName=os.path.join(pybullet_data.getDataPath(), model_urdf_directory_extension), flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

#print(os.getcwd())
#p.setAdditionalSearchPath(os.getcwd())
octopusBodyUniqueId = p.loadURDF( fileName=os.path.join(pybullet_data.getDataPath(), model_urdf_directory_extension), flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

#enables gravity
p.setGravity(0,0,0)

#sets real time simulation
p.setRealTimeSimulation(enableRealTimeSimulation=1) #now we dont have to call p.stepSimulation() in order to advance the timestep of the simulation environment

#turn off all motors so that joints are not stiffened for the rest of the simulations
p.setJointMotorControlArray(
    bodyUniqueId=octopusBodyUniqueId, 
    jointIndices=list(range(number_of_links_urdf)), 
    controlMode = p.POSITION_CONTROL, 
    positionGains=[0.1]*number_of_links_urdf, 
    velocityGains=[0.1]*number_of_links_urdf, 
    forces=[0]*number_of_links_urdf
)

#load the block thatwe are trying to avoid


targetPositions=[0]*(number_of_links_urdf) ; targetPositions[0]=3.14*(36/360) ; #set desired joint angles in radians  
targetVelocities=[0]*(number_of_links_urdf) ;  # set desired joint velocities  

#move the robot according to desited joint angles and desired joint velocities using PD control
p.setJointMotorControlArray(
    bodyUniqueId=octopusBodyUniqueId, 
    jointIndices=list( range(number_of_links_urdf) ), 
    controlMode=p.PD_CONTROL, 
    targetPositions=targetPositions, 
    targetVelocities=targetVelocities, 
    positionGains=[1]*number_of_links_urdf, 
    velocityGains=[1]*number_of_links_urdf, 
    forces=[100]*number_of_links_urdf 
)

# overshoot

j=0
while(1):
    x=1 #dummy assignment so that simulation runs indefinitely
    a=p.getJointStates( bodyUniqueId=octopusBodyUniqueId, jointIndices=list( range(number_of_links_urdf) ) )   
    j = j+1
    if (j%5e4==0):     
        j=0
        print('')
        print('joint info: ')
        print('joint positions :' , a[0] )
        print('joint velocities : ', a[1])
        print('joint reaction forces : ', a[2])
        print('applied motor torque : ', a[3])


