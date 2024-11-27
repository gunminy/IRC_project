Open 4 terminals and navigate to catkin_ws.
Run each line in each terminal

roscore
rosrun face_position face_position_publisher.py
roslaunch open_manipulator_controller open_manipulator_controller.launch
rosrun face_position face_position_subscriber.py
