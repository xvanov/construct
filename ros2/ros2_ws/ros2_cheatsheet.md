
# Packaging Guide ROS2

1. create a package
This guide outlines the steps for packaging `example36_pkg` in ROS2.

## Steps for Packaging

### 1. Create a Package

To create a new package, use the following command:

```bash
ros2 pkg create --build-type ament_python example36_pkg --dependencies rclpy std_msgs geometry_msgs custom_interfaces
```

2. create a launch file
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example36_pkg',
            executable='example36',
            output='screen'),
    ])
```

3. modify setup.py 
```python
from setuptools import setup
import os
from glob import glob

package_name = 'example36_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'example36 = example36_pkg.example36:main'
        ],
    },
)
```
4. compile package
```bash
colcon build --packages-select example36_pkg
source ~/ros2_ws/install/setup.bash
```



