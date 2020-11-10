import argparse

#state the number of links in your urdf (edit this as necessary)
if __name__ == '__main__':
    parser=argparse.ArgumentParser(description='example script to demonstrate pybullet simulator and control of a 2D N-link robot')  #add capability of user input
    parser.add_argument('-num_links', '--number_of_links_urdf', help='number of links in urdf', type=int, default=1 )  # set default input to 1 if none is given
    arguments=parser.parse_args()
    print('type: ', type(arguments.number_of_links_urdf))
    number_of_links_urdf=int(arguments.number_of_links_urdf) #define the number of links


import inspect #used to list attributes of objecte
import os

import pybullet_data

home_dir = os.getenv("HOME")


#path_to_xml_file = os.path.join(home_dir, "Documents/RoboLab/bullet3/examples/pybullet/gym/pybullet_data/romans_urdf_files/octopus_files/base_plane.urdf")
path_to_xml_file = os.path.join(os.getcwd(), "base_plane.urdf") 


import xml.etree.ElementTree as ET
tree = ET.parse(path_to_xml_file)
root = tree.getroot()

print(root, root.tag, root.attrib)
print(inspect.getmembers(root))
print()
print(list(root))

print("arguments take by subelement")
print(ET.SubElement)

#args_for_subelement = {'parent':'root', 'tag':'link','attrib':'{'name': 'link_1'}'}

number_of_links = int(number_of_links_urdf) ######################## CHANGE THE NUMBER OF LINKS HERE
# information on link properties 
# http://wiki.ros.org/urdf/XML/link
for link_index in range(1, number_of_links+1):
	link = ET.SubElement( root, 'link', {'name': 'link_' + str(link_index)}  )
	  
	
        #collision
	collision = ET.SubElement(link, 'collision' ) #collision = ET.SubElement(link, 'collision' , {'': '' } )
	
	geometry = ET.SubElement( collision, 'geometry' ) #geometry = ET.SubElement( collision, 'geometry', {'': '' } )
	cylinder = ET.SubElement( geometry, 'cylinder', {'length':'2.1', 'radius':'0.1'} )    

	origin = ET.SubElement( collision, 'origin', {'rpy':'0 0 0', 'xyz':'0 0 1'} )


	#inertial
	inertial = ET.SubElement(link, 'inertial')
	
	origin = ET.SubElement(inertial, 'origin', {'xyz':'0 0 1'})
	mass = ET.SubElement(inertial, 'mass', {'value':'1'} )
	inertia = ET.SubElement(inertial,'inertia',{'ixx':'1.0', 'ixy':'0.0', 'ixz':'0.0', 'iyy':'1.0', 'iyz':'0.0', 'izz':'1.0'} ) 

# information on joint properties 
# http://wiki.ros.org/urdf/XML/joint#Attributes	
for joint_index in range(1, number_of_links + 1):
	joint = ET.SubElement( root, 'joint' , {'name':'link_'+ str(joint_index-1) + '_to_link_'+ str(joint_index) , 'type':'continuous'} ) 
	#joint = ET.SubElement( root, 'joint'+ str(joint_index-1) + 'link'+ str(joint_index) , {'name':'link_'+ str(joint_index-1) + '_to_link_'+ str(joint_index) , 'type':'continuous'} )
	
	parent = ET.SubElement( joint , 'parent', {'link':'link_'+str(joint_index-1)} )
	child = ET.SubElement( joint , 'child', {'link':'link_'+str(joint_index)} )
	origin = ET.SubElement( joint , 'origin', {'xyz':'0 0 2'} )
	axis = ET.SubElement( joint , 'xyz', {'xyz':'1 0 0'} )
	dynamics = ET.SubElement(joint, 'dynamics', {'damping':'1.0', 'friction':'10.0'} )
	
#tree structure is as follows
# #robot(root)
	##link
		###collision
			####geometry
				#####cylinder
			####geometry
			####origin
		###collision
	##link
	##joint
		###parent
		###child
		###origin
	##joint
# #robot	

#ET.SubElement( args_for_subelement )



"""
for i in root:
        for j in i:
                for k in j:
                        for l in k:
                                print(l, k, i, j)
"""
name_of_generated_xml_file = "octopus_generated_"+ str(number_of_links) + "_links.xml"
name_of_generated_urdf_file = "octopus_generated_"+ str(number_of_links) + "_links.urdf"

#changes the current working directory to the pybullet_data base DataPath
os.chdir( pybullet_data.getDataPath() ) 

if not os.path.isdir( "cs292c_robot_models"):  #check if directory doesn'y already
  os.mkdir("cs292c_robot_models") #makes a new directory

os.chdir( os.path.join(pybullet_data.getDataPath(), "cs292c_robot_models") )  #go into newly made directory


#writes the new file to the pybullet_data base DataPath
tree.write( name_of_generated_xml_file )
tree.write( name_of_generated_urdf_file )

print()

