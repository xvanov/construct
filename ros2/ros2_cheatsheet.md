
# Packaging Guide ROS2

1. create a package
This guide outlines the steps for packaging `example36_pkg` in ROS2.

## Steps for Packaging

### 1. Create a Package

To create a new package, use the following command:

```bash
ros2 pkg create --build-type ament_python example36_pkg --dependencies rclpy std_msgs geometry_msgs 
ros2 pkg create my_action_server --build-type ament_python --dependencies rclpy rclpy.action
```

2. create a launch file (in launch directory)
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
5. run
```bash
ros2 launch example36_pkg example36.launch.py
```

# Create a custom interface

1. Create a directory named `msg`/`srv`/`action` inside your package
```
ros2 pkg create --build-type ament_cmake custom_interfaces --dependencies rclcpp std_msgs
ros2 pkg create --build-type ament_cmake custom_interfaces --dependencies rclcpp action_msgs
```

2. Inside this directory, create a file named `name_of_your_message.msg`/`.srv`/`.action`
	ex.

int32 year
int32 month
int32 day

3. Modify the `CMakeLists.txt` file
	ex. add this line:
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
  DEPENDECIES geometry_msgs
)
4. Modify package.xml file
	For msg/srv:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

For actions:
```xml
<depend>rosidl_default_generators</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

5. Compile and source
```bash
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

6. Use in your node
ros2 interface show custom_interfaces/msg/Age


### 1. Calls

Subscribe to Topic
```bash
```


Publish to Topic
```bash
```

Request to Server
```bash
```

Request to Action
```bash
ros2 action send_goal <action_name> <action_type> <values>

```

