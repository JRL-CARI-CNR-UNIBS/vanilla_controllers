# PID effort controller
The pid_effort_controller is a PID (Proportional Integral Derivative) controller implementation for controlling the effort (force/torque) of a robot joint. The code is part of the vanilla_controllers repository, which is a collection of basic robot controllers for various tasks.

### Purpose
The purpose of the pid_effort_controller is to control the effort of a robot joint in order to achieve a desired behavior or task. The controller receives feedback from the joint sensor and generates a control signal to adjust the effort of the joint based on the error between the desired and actual effort.

### Features
The controller is implemented using a PID control algorithm, which is a commonly used feedback control mechanism in robotics. It allows the controller to take into account the current error (proportional), the history of the error (integral), and the predicted future error (derivative) in order to generate a control signal that minimizes the error and achieves the desired behavior.

The pid_effort_controller code includes functions for computing the proportional, integral, and derivative gains, as well as the control signal based on the current error and the gains. It also includes functions for setting and getting the desired effort, the joint position and velocity, and the current effort and position of the joint.

### Usage
The pid_effort_controller code can be used in various robot applications that require controlling the effort of a joint. The user can modify the gains and other parameters of the controller to adjust the control behavior based on the specific requirements of the application.

### Dependencies
The pid_effort_controller code has dependencies on the ROS (Robot Operating System) framework and the controller_interface and hardware_interface libraries, which are commonly used in ROS-based robot controllers. The code is designed to work with a variety of robot hardware platforms and can be configured to work with different types of joint sensors and actuators.

### Contributors
The sea_effort_controller is developed and maintained by the Joint Research Laboratory (JRL) at the University of Brescia (UNIBS) and the National Research Council of Italy (CNR). Contributions to the code are welcome and can be submitted via pull requests to the vanilla_controllers repository.
