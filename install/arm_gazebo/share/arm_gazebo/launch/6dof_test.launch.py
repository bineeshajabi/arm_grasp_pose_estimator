'''
Author: Bineesha
Date: 07 / 10 /2025

Describer:  Launch the 6 DOF robotic arm and spawn with controls in Gazebo

'''
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
import os


def generate_launch_description():
   
   pkg_arm ='arm_description'
   urdf_file='arm_model.urdf.xacro'
   rviz_file='arm_6dof.rviz'

   pkg_description=FindPackageShare(pkg_arm)   

   # Initialization of directories
   xacro_file_GZ =PathJoinSubstitution([pkg_description,'urdf','robots',urdf_file]) 

   rviz_config_file=PathJoinSubstitution([pkg_description,'rviz',rviz_file])


   robot_description_content=ParameterValue(Command(['xacro',' ',xacro_file_GZ]),
                                            value_type=str)
   
   robot_controllers=PathJoinSubstitution([FindPackageShare('arm_gazebo'),'config','simple_controller.yaml'])


   # Robot State Publisher to generate the /robot state topic with URDF data
   robot_state_publisher = Node(package   ='robot_state_publisher',
								 executable ='robot_state_publisher',
								 name       ='robot_state_publisher_arm',
								 output     ='screen',
								 parameters =[{'robot_description': robot_description_content}]     
								)
   
   control_node=Node(
          package='controller_manager',
          executable='ros2_control_node',
          parameters=[{'robot_description': robot_description_content},
                      robot_controllers],
          output='both'
     )
   
   delayed_robot = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[control_node],
        )
    )
     
   
   joint_state_broadcaster_spawner=Node(
          package="controller_manager",
          executable="spawner",
          arguments=['joint_state_broadcaster']
     )
   
   controller_spawner=Node(
          package="controller_manager",
          executable="spawner",
          arguments=["arm_controller","--param-file", robot_controllers]
     )    
   
   rviz_arm_node=Node(
		 executable ='rviz2',
		 name       ='rviz2_arm',
		 output     ='log',
		 arguments  =['-d', rviz_config_file])
     
   
   '''
   #Spawn the robot
   spawn_entity_robot=Node(
        package='ros_gz_sim',
        executable='create',
        arguments   = ['-topic', 'robot_description' , '-name ', 'arm_description'],
        output='screen'
    )
   
   #Declaration Gazebo and world
   world_file_name = 'empty_world.sdf'
   world = os.path.join(get_package_share_directory('arm_gazebo'), 'world', world_file_name)

   #Create the gazebo world
   gazebo_node = ExecuteProcess(
        cmd=['gz' , 'sim', '--verbose', world],
          output='screen')
   '''
        
   return LaunchDescription([
        robot_state_publisher,
        delayed_robot,
        joint_state_broadcaster_spawner,
        controller_spawner,
        rviz_arm_node
                ])
   
   

