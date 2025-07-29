# Ctrl-Alt-Defeat-WRO-Future-Engineers-2025

Welcome to the repository for our entry in the WRO Future Engineers 2025 competition!
We are a team of 2 aspiring engineers building an autonomous robotic car using Raspberry Pi 5 & RP2040, equipped with various sensors and custom designs.

## The Team
Adbhut, Pranav

## Project Overview
This project is our official entry for the Future Engineers category at the World Robot Olympiad 2025. Our goal is to construct a self-driving vehicle capable of navigating the track in both the Open and Obstacle Rounds. The programs are written in C++ and Python languages. This repository contains the programs, hardware description and design files of our model.

## Hardware Components
- __Compute:__ Raspberry Pi 5 (main computer), Raspberry Pi 2040 (Pico, real-time control)
- __Sensors:__ 1D and 2D LiDAR, IMU , rotary encoders motor, Picamera
- __Actuators:__ N20 DC gear motor (with encoders), MG996R 180° servo (steering)
- __Chassis:__ Commercially-available base, 3D-printable modifications (Links/STL files included)
- __Electronics:__ Custom peripherals board PCB for easy connections and sensor breakout

## Project Structure
- `Design Files` : The Bill of Materials, 3D-printable design files, hardware etc. are included along with pictures and description of the robot and components
- `Initial Tests` : Intial tests on various sensors to ensure accurate data collection and basic object detection algorithm
- `hw` : 
- `sw` : 
- `Open Round` : It contains all the program files for the open round over various iterations and hardware setups.
- `Obstacle Round` : 

## System Architecture 
-	Raspberry Pi 5:  Runs high-level logic like lane and object detection, route planning, 2D LiDAR and main robot state machine.
-	Raspberry Pi 2040 (Pico):  Handles real-time tasks (motor PID, encoder feedback, servo control, IMU, 1D LiDAR).
-	LiDARs:  Used for detecting direction, obstacles and parking area
-	IMU:  Gives yaw (and other orientations) of the robot in [0,360).
-	Motor/Servos:  PWM and PI controls respectively for movement using sensor data, coordinated by the RP2040.
