#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # create a place holder for launch description
    launch_description = LaunchDescription()

    ### Example for adding launch argument ###
    rate = LaunchConfiguration('rate')
    rate_launch_arg = DeclareLaunchArgument('rate',default_value='5.0')
    launch_description.add_action(rate_launch_arg)
    
    #launch Turtlesim Node
    TurtlesimNode = Node(
    package='turtlesim',
    executable='turtlesim_node'
    
    )
    launch_description.add_action(TurtlesimNode)

    #launch Linear NoiseGenerator Node
    LinearNoiseGenerator = Node(
        package='fra333_lab1_7',
        executable='noise_generator.py',
        namespace= 'linear',
        arguments=[rate]
    )
    launch_description.add_action(LinearNoiseGenerator)

    #launch Angular NoiseGenerator Node
    AngularNoiseGenerator = Node(
    package='fra333_lab1_7',
    executable='noise_generator.py',
    namespace= 'angular',
    arguments=[rate]
    )
    launch_description.add_action(AngularNoiseGenerator)

    #launch VelocityMux Node
    VelocityMuxx = Node(
    package='fra333_lab1_7',
    executable='velocity_mux.py',
    arguments=[rate]
    )
    launch_description.add_action(VelocityMuxx)

    ## Example for execute a shell command in python script ###
    # vx = 1.0
    # wz = 1.0
    # pub_cmd_vel = ExecuteProcess(
    #     cmd = [[f'ros2 topic pub -r 10 /turtle1/cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: {vx}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}"']],
    #     shell=True
    # )
    # launch_description.add_action(pub_cmd_vel)
    
    return launch_description

    