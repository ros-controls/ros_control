# Hardware Interfaces

Hardware interfaces are used by ROS control in conjunction with one of the
available ROS controllers to send (\ref hardware_interface::RobotHW::write)
commands to the hardware and receive (\ref hardware_interface::RobotHW::read)
states from the robot's resources (joints, sensors, actuators).

A list of available hardware interfaces (provided via the HardwareResourceManager)
as of this writing are:

- [JointCommandInterface](include/hardware_interface/joint_command_interface.h): 
  hardware interface to support commanding and reading the state of an array of
  joints. Note that these commands can have any semantic meaning as long as each
  can be represented by a single double, they are not necessarily effort commands. 
  To specify a meaning to this command, see the derived classes:
  - [EffortJointInterface](https://github.com/ros-controls/ros_control/blob/c6ee2451cf919307b7c1dbc75b32bec7d1b52d23/hardware_interface/include/hardware_interface/joint_command_interface.h#L82): 
    for commanding and reading effort-based joints.
  - [VelocityJointInterface](https://github.com/ros-controls/ros_control/blob/c6ee2451cf919307b7c1dbc75b32bec7d1b52d23/hardware_interface/include/hardware_interface/joint_command_interface.h#L85): 
    for commanding and reading velocity-based joints.
  - [PositionJointInterface](https://github.com/ros-controls/ros_control/blob/c6ee2451cf919307b7c1dbc75b32bec7d1b52d23/hardware_interface/include/hardware_interface/joint_command_interface.h#L88): 
    for commanding and reading position-based joints.
- [JointStateInterfaces](include/hardware_interface/joint_state_interface.h): 
  hardware interface to support reading the state of an array of named joints, 
  each of which has some position, velocity, and effort (force or torque).
- [ActuatorStateInterfaces](include/hardware_interface/actuator_state_interface.h): 
  hardware interface to support reading the state of an array of named actuators,
  each of which has some position, velocity, and effort (force or torque).
- [ActuatorCommandInterfaces](include/hardware_interface/actuator_command_interface.h)
  - [EffortActuatorInterface](https://github.com/ros-controls/ros_control/blob/c6ee2451cf919307b7c1dbc75b32bec7d1b52d23/hardware_interface/include/hardware_interface/actuator_command_interface.h#L79)
  - [VelocityActuatorInterface](https://github.com/ros-controls/ros_control/blob/c6ee2451cf919307b7c1dbc75b32bec7d1b52d23/hardware_interface/include/hardware_interface/actuator_command_interface.h#L82)
  - [PositionActuatorInterface](https://github.com/ros-controls/ros_control/blob/c6ee2451cf919307b7c1dbc75b32bec7d1b52d23/hardware_interface/include/hardware_interface/actuator_command_interface.h#L85)
- [PosVelJointInterface](/include/hardware_interface/posvel_command_interface.h)
- [PosVelAccJointInterface](/include/hardware_interface/posvelacc_command_interface.h)
- [Force-torque sensor Interface](include/hardware_interface/force_torque_sensor_interface.h)
- [IMU sensor Interface](/include/hardware_interface/imu_sensor_interface.h)

Note that \ref hardware_interface::JointCommandInterface allows both reading
joint state and commanding [effort|velocity|position]-based joints
(see this [answer](https://answers.ros.org/question/209619/differences-between-hardware-interfaces/?answer=209636#post-id-209636)).