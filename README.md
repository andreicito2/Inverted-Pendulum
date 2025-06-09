# Single-Inverted-Pendulum
Repository for our ENGR 454 Control Systems Project

Classic inverted Pendulum using modern control theory with Octave.

![!\[alt text\](ENGR_454_Single_invpen.jpg)](Photos/ENGR_454_Single_invpen.jpg)

Video: https://youtu.be/5oP2OeEzNEM

## Description:
 This project was done for our Control Systems class, in which we where tasked with developing a control system using the methods learned during the class. The project chosen was to balance an inverted pendulum with the possibility of adding a second pendulum. The system uses a Spartan-3 FPGA for communication with a DC motor, 4 encoders, and 2 hall effect sensors. The DC motor moves the cart on the track with a string. The encoders provide the angles of the 2 pendulums as well as the position of the cart. One encoder remains unused. The hall effect sensors are intended to shutoff the FPGA when the cart reaches them, preventing the cart from slamming into the end supports. A laptop, running Octave, uses the "sockets" package to communicate with the FPGA board.

## Background:
 This apparatus has been sitting in a lab in our university for quite a few years. It has been used as a cool demonstration for new students during tours of the lab. However, the previous code was very bulky and hard to read. Additionally, there was almost no documentation about the system. We decided to fully rewrite the code as well as writing detailed, easy to follow documentation to assist future students in understanding feedback and controls.

## Setup:
 To use the system, the "pendulum_single.m" and "ctrlbox.m" files are required. The "pendulum_single.m" file holds the control method for balancing a the pendulum. The "ctrlbox.m" file interfaces with the FPGA to operate the motor.

## Features:
 - Full State Observer
 - LQR Control using the Octave "control" package
 - Octave

## Control Approach:
 Our Project uses a state-space feedback controller designed using modern control theory, using:

 - Real-time state estimation with a Luenberger observer
 - Robust control tuning using matrix-based gain design
 - Manual tuning of observer poles for optimal performance

## Difficulties Encountered
 - Had to derive equations of motions multiple times due to sign errors and incorrect methods
 - Issues with understanding use of the "ctrlbox" functions, as there was limited documentation available
 - Would sometimes stop working with no change to code, then randomly start working again
 - Unable to successfully implement second pendulum within time limit

## Future Work:
 - Perfecting Second Pendulum
 - Removing resonance spots 
 - Recentering the cart after it drifts because of balancing
 - Removing drift while balancing
 - Overhaul of system to convert to Arduino or other modern solution

## Contributors:
 - Tyler Sing
 - Andrei Maiorov
