from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wall_follower_node = Node(
        package='wall_follower',
        executable='wall_following',
        name='wall_follower'
    )

    wall_finder_node = Node(
        package='wall_follower',
        executable='wall_finder',
        name='wall_finder'
    )

    odom_recorder_node = Node(
        package='wall_follower',
        executable='odom_recorder',
        name='odom_recorder'
    )

    return LaunchDescription([
        wall_finder_node, 
        odom_recorder_node,
        wall_follower_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
