'''
Author: Bineesha
Date: 06 / 10 /2025

Describer:  Launch the 6 DOF robotic arm and visualise in RViz
			
'''
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
	
	'''
	Generate the launch file for visualization of the robot model in RViz
	- Robot state publisher node : Broadcast the transform 
	- Joint state publigher gui : Simulate the joints with the publishesd topic /joint_states
	- RViz : Visualize the robot	
	'''

	#Define filenames
	pkg_arm = 'arm_description'
	urdf_desc_file='arm_model.urdf.xacro'
	rviz_file = 'arm_6dof.rviz'

	#Set the paths
	pkg_descripton = FindPackageShare(pkg_arm)

	urdf_path=PathJoinSubstitution([pkg_descripton,'urdf','robots',urdf_desc_file])

	rviz_config_file=PathJoinSubstitution([pkg_descripton,'rviz',rviz_file])

	robot_description_content=ParameterValue(Command([
		'xacro',' ',urdf_path]),
		value_type=str)
	
	# Subscribe to the joint states of the robot, and publish the 3D pose of each link.
	robot_state_publisher_cmd=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_arm',
		output='screen',
		parameters=[{
			'robot_description':robot_description_content}]
	)
	
	# Publishes the joint states from URDF file 
	joint_state_publisher_gui_arm=Node(
		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		name='joint_state_publisher_gui_arm',
		output='screen'		
	)

	#RViz node 
	rviz_arm_node=Node(
		 executable ='rviz2',
		 name       ='rviz2_arm',
		 output     ='log',
		 arguments  =['-d', rviz_config_file])



	return LaunchDescription([robot_state_publisher_cmd, joint_state_publisher_gui_arm, rviz_arm_node ])

