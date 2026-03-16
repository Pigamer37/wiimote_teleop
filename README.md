# wiimote_teleop
 Ros2 humble package for teleoperating a 6DOF robot with a wiimote
# Installation
1. Clone the repository and source your ROS workspace
2. Install dependencies:
```bash
rosdep install --from-paths 
```
3. Compile/build:
```bash
colcon build --symlink-install
```
or
```bash
colcon build --symlink-install --packages-select wiimote-teleop
```
to only compile the package (not all packages in your ROS workspace)
Then source:
```bash
source install/setup.bash
```
Also build and source Delto_grippers
4. [Take a look at the wiimote package's preliminary tests](https://github.com/ros-drivers/joystick_drivers/blob/ros2/wiimote/doc/testing.md)

5. Run (with simulation):
```bash
ros2 launch wiimote_teleop full.launch.py
```
Launch parameters:
- end_link: the string name of the last link (usually "tool0" by convention, the default value)
- rviz: launch rviz to visualize the robot (eiter true or false)
- wiimote: use the wiimote_handler or a generic 2 axis controller (joystick) handler to command the robot (eiter true or false)
# Acknowledgements
This package uses the description from the 6DOF robot from ros2_control's example 7