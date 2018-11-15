source /opt/ros/kinetic/setup.bash
cd /home/developer/catkin_ws/
catkin_make
catkin init

cd /home/developer/catkin_ws/
catkin_make --pkg astra_camera
cd /home/developer/catkin_ws/
catkin_make --pkg astra_launch

source /home/developer/catkin_ws/devel/setup.bash
