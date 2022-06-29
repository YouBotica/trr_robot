from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
import launch.actions
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # Constants for paths to different files and folders
    gazebo_models_path = 'models'
    package_name = 'robot'
    robot_name_in_model = 'robot'
    rviz_config_file_path = 'rviz/config_rviz.rviz'
    urdf_file_path = 'urdf/trr_sw.urdf'
    world_file_path = 'worlds/trr_world.world'
     
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.00'

    urdf_file_name = 'trr_sw.urdf'
    urdf = os.path.join(
        get_package_share_directory('robot'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    # Launch the robot
    spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
                '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')

    move_joints = Node(
    package='robot',
    executable='move_joints_gazebo',
    output='screen',
    arguments=["0.0"])

    state_publisher = Node(
    package='robot',
    executable='state_publisher',
    name='state_publisher',
    output='screen')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
    arguments=[urdf])

    # Publish the joint states of the robot
    start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    )
    #condition=UnlessCondition(gui))

    return LaunchDescription([
        spawn_entity_cmd,
        load_joint_state_controller,
        load_joint_trajectory_controller,
        launch_ros.actions.Node(
            package='robot',
            executable='move_joints_gazebo',
            output='screen',
            arguments=["1.0"]),

        launch_ros.actions.Node(
            package='robot',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
    ])