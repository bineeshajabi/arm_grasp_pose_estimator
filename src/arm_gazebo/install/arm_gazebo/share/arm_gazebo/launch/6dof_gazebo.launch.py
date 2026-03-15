'''
Author: Bineesha
Date: 07 / 10 /2025

Describer:  Launch the 6 DOF robotic arm and spawn with controls in Gazebo

'''

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess , IncludeLaunchDescription , SetEnvironmentVariable
from launch.launch_description_sources import  PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
import xacro

def generate_launch_description():

    pkg_description ='arm_description' 
    # Initialization of directories
    xacro_file_GZ =os.path.join(get_package_share_directory(pkg_description),'urdf','arm_complete.urdf.xacro')
       
    #Declaration Gazebo and world
    world_file_name = 'empty_world.sdf'
    world = os.path.join(get_package_share_directory('arm_gazebo'), 'world', world_file_name)
   
   # Robot State Publisher to generate the /robot state topic with URDF data
    robot_state_publisher = Node(package    ='robot_state_publisher',
								 executable ='robot_state_publisher',
								 name       ='robot_state_publisher_arm',
								 output     ='screen',
								 parameters =[{'robot_description_arm': Command(['xacro',' ', xacro_file_GZ])           
								}])
    
    #Create the gazebo world
    gazebo_node = ExecuteProcess(
        cmd=['gz' , 'sim', '--verbose', world],
          output='screen')
    
    #Spawn the robot
    spawn_entity_robot=Node(
        package='ros_gz_sim',
        executable='create',
        arguments   = ['-topic', 'robot_description_arm' , '-name ', 'arm_description'],
        output='screen'
    )
        
        
    
    return LaunchDescription([
        robot_state_publisher,
        spawn_entity_robot, 
        gazebo_node
        ])

