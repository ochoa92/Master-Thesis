##  GAZEBO 
# Start the controllers using roslaunch
1) TERMINAL 1:
roslaunch thesis_control j2n6s300_gazebo.launch

# nodes
2) TERMINAL 2:
rosrun thesis_demo imp_control.py

# Use RQT To Send Commands
3) rosrun rqt_gui rqt_gui

# To launch moveIt with Gazebo
4) roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch



##  ROBOT 
# 1) TERMINAL 1:
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300 use_urdf:=true

**use_urdf** specifies whether the kinematic solution is provided by the URDF model. This is recommended and is the default option.

# 2) TERMINAL 2:
rosservice call /j2n6s300_driver/in/home_arm
**ENVIA ROBOT PARA A POSIÇÃO HOME**

# 3) TERMINAL 3:
# To visualize a list of the available topics
rostopic list

# To get info about a specific topic
rostopic info /j2n6s300_driver/'{...}'

## Use RVIZ to vizualize
rosrun rviz rviz
# 1) Fixed Frame: root
# 2) Add: RobotModel
# 3) Add: Effort

## MOVEIT
roslaunch j2n6s300_moveit_config j2n6s300_demo.launch

## Finger position control
# The unit of finger command can be by '{turn | mm | percent}'
rosrun kinova_demo fingers_action_client.py j2n6s30 percent -- 100 100 100

## Torque Control
rosservice call j2n6s300_driver/in/set_torque_control_parameters

# Switch to Torque control: 1 | Position control: 0
rosservice call j2n6s300_driver/in/set_torque_control_mode 1

# Publish torque commands
rostopic pub -r 100 /j2n6s300_driver/in/joint_torque kinova_msgs/JointTorque "{joint1: 0.0, joint2: 0.0, joint3: 0.0, joint4: 0.0, joint5: 0.0, joint6: 1.0}"

# To visualize a list of the available services
rosservice list

# And to get info about a specific service
rosservice info /j2n6s300_driver/'{...}'

# nodes
rosrun thesis_demo imp_control_robot.py

## Calibration Robot
rosrun thesis_demo calibration_robot.py

# To acess API Funcs:
rosrun kinova_driver kinova_api_funcs
rosrun thesis_demo kinova_api_wrapper.py
