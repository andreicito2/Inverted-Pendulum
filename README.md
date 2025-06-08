# Single-Inverted-Pendulum
*to Properly observe Derivations download IPYNB file to properly compile

Repository for our ENGR 454 Control Systems Project

Classic inverted Pendulum using modern control theory with Octave.

![!\[alt text\](ENGR_454_Single_invpen.jpg)](Photos/ENGR_454_Single_invpen.jpg)

Video: https://youtu.be/5oP2OeEzNEM

## Description:
This project is a part of our Control Systems class, where we where tasked with developing a control system with the methods we learned in the first half of the class. Our project was to make a single inverted pendulum with the possibility of adding a second inverted pendulum. Then system uses a Spartin-3 FPGA for communication with an dc motor, 2 encoders, and 2 hall effect sensors.
The DC motor moves the cart on the track with a string and the encoders give the angles of the 2 pendulums. and the hall effect sensors are their to keep the motor from launching the cart in to the end of the track and the motor shaft from snapping. 
The FPGA uses Octave and the "sockets" library to communicate with a laptop. Then the code uses the "control" library for LQR placement. 

## Setup:
To use the system you need the "ctrlbox.m" file and "pendulum_single.m" files. The "ctrlbox.m" file talks with with FPGA to run code and get error messages back. The "pendulum_single.m" file holds the control method for balancing a single inverted pendulum.


## Features:
- Full State Observer
- LQR Control 
- Octave


## Control Approach:
Our Project uses state-space feedback controller designed using modern control theory, using:

- Real-time state estimation
- Robust control tuning using matrix-based gain design
- Manual gain tuning

## Future Work:
 - Adding Second Pendulum
 - Removing resonance spots 
 - Recentering the cart after it drifts because of balancing
 - Removing drift while balancing

## Contributors:
Tyler Sing
Andrei Maiorov
