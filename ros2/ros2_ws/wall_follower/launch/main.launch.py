from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wall_follower_node = Node(
        package='wall_follower',  # Replace with your package name
        executable='wall_following',
        name='wall_follower'
    )

    wall_finder_node = Node(
        package='wall_follower',  # Replace with your package name
        executable='wall_finder',
        name='wall_finder'
    )

    return LaunchDescription([
        wall_follower_node,
        wall_finder_node
    ])

if __name__ == '__main__':
    generate_launch_description()
