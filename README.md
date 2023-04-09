# Lab1
### ROS2 node for controlling a turtlesim bot using a keyboard.

## Installation
Clone this repository into your ROS2 workspace and build it using colcon build:
```
cd ~/ros2_ws/src
git clone https://github.com/RybOlya/Lab1.git
cd ..
colcon build
```
## Usage
Launch the turtle_controller.launch.py file to start the turtlesim node and the turtle controller node:
```
ros2 launch my_turtle_controller turtle_controller.launch.py

```
By default, the turtle is controlled using the 'g', 'd', 'f', 'p', and 'l' keys for going forward, turning left, turning right, putting the pen up, and putting the pen down, respectively. You can override these keys by creating a controller.yaml file in the params directory and modifying the keys.
## License
This project is licensed under the MIT License - see the LICENSE file for details.