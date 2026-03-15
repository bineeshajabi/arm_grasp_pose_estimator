from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler,TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description(): 

    delay_time = 3.0

    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')
    
    # Start arm controller
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen')
       
    
    delayed_start =TimerAction(
        period=delay_time,
        actions=[start_joint_state_broadcaster_cmd,start_arm_controller_cmd]
    )
   
 
    return LaunchDescription([
        delayed_start
    ]
        
    )
