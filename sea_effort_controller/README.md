# LQR Control of a Series Elastic Actuator (SEA)

This repository contains a LQR (Linear Quadratic Regulator) controller implementation for controlling the effort of a series elastic actuator (SEA) in a robot. The controller is implemented using the controller_interface::MultiInterfaceController interface and adds an integrator state to improve the control performance.

## Purpose
The purpose of the sea_effort_controller is to control the effort of a series elastic actuator in order to achieve a desired behavior or task. The controller receives feedback from the actuator sensor and generates a control signal to adjust the effort of the actuator based on the error between the desired and actual effort.

## Features
The controller is implemented using a LQR control algorithm, which is a commonly used feedback control mechanism in robotics. It allows the controller to take into account the current error, as well as the predicted future error, in order to generate a control signal that minimizes the error and achieves the desired behavior.

The sea_effort_controller code includes functions for computing the control gains using the LQR algorithm, as well as the control signal based on the current error, the gains, and the integrator state. It also includes functions for setting and getting the desired effort, the actuator position and velocity, and the current effort and position of the actuator.

## Usage
The sea_effort_controller code can be used in various robot applications that require controlling the effort of a series elastic actuator. The user can modify the gains and other parameters of the controller to adjust the control behavior based on the specific requirements of the application.

## Dependencies
The sea_effort_controller code has dependencies on the ROS (Robot Operating System) framework and the controller_interface and hardware_interface libraries, which are commonly used in ROS-based robot controllers. The code is designed to work with a variety of robot hardware platforms and can be configured to work with different types of actuator sensors and interfaces.

## Contributors
The sea_effort_controller is developed and maintained by the Joint Research Laboratory (JRL) at the University of Brescia (UNIBS) and the National Research Council of Italy (CNR). Contributions to the code are welcome and can be submitted via pull requests to the vanilla_controllers repository.
