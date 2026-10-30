from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    switch_node = Node(
        package='umrt_com_antenna',
        executable='switch_publisher_node',
        name='switch_publisher_node',
        output='screen',
    )

    imu_node = Node(
        package='umrt_com_antenna',
        executable='qmc5883p_node',
        name='qmc5883p_node',
        output='screen',
        parameters=[{'i2c_bus': '/dev/i2c-1'}],
    )
    
    antenna_node = Node(
        package='umrt_com_antenna',
        executable='antenna_rotation',
        name='antenna_rotation',
        output='screen'
    )
    '''
    pid_node = Node(
        package='controller_manager',
        executable='spawner',
        argument=['joint_pid_controller'],
        remappings=[
                ('~/reference', '/input')
        ],
        output='screen'
    )
    '''
    return LaunchDescription([switch_node, imu_node, antenna_node]) # removed pid_node for now
