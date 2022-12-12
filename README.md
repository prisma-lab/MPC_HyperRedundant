# Stabilization and control on a pipe-rack of a wheeled mobile manipulator with a snake-like arm

The following repository contains the implementation of the inspection task completed by a snake-like arm mounted on a wheeled rover described in the paper actually submitted to RA-L journal.

## Prerequisites
The Offline navigation function algorithm is written in MATLAB programming language with its version 2019b.

To run the control node it is needed to install the full version of ROS Noetic http://wiki.ros.org/noetic/Installation.
The controller receive the setpoints to compute the desired trajectory offline and than start the control loop.

The MPC is implemented with the ACADOS library in MATLAB and than converted to C code. 
To install Acados on your PC follow the guide here https://github.com/acados/acados

To compile the ros node:
Create a ros workspace and copy in src the folder prisma_snake_control, prisma_snake_support_gazebo and industrial_environment.

Open a terminal and type:
```
catkin build prisma_snake_control prisma_snake_support_gazebo industrial_environment
```

To run the code:
Launch the simulation on Gazebo in a terminal:
```
roslaunch prisma_snake_support_gazebo prisma_snake.launch 
```

Launch MATLAB simulation to compute setpoints:
```
traj_sender_to_cpp.m
```

In another terminal navigates in the forlder with the matlab code of the mpc algorithm and type:
```
source env.sh
```
In the same terminal launch the control node:
```
rosrun prisma_snake_control prisma_snake_final 
```
Follow the instruction on the screen. When the string "TRAJECTORY CORRECTLY PLANNED" appears press play on Gazebo.

## References
Code developed by Simone D'Angelo and Antonio Corrado.
