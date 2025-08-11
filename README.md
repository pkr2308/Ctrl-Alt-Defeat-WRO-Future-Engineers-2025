# Ctrl-Alt-Defeat-WRO-Future-Engineers-2025

Welcome to the repository for our entry in the WRO Future Engineers 2025 competition!
We are a team of 2 aspiring engineers building an autonomous robotic car using Raspberry Pi 5 & RP2040, equipped with various sensors and custom designs.

## About the Team
Adbhut Patil: <TODO>

Pranav Kiran Rajarathna: Pranav is currently sudying in the 11th grade(PCMC combination) and has been interested in Robotics for several years . He has participated in WRO in other categories in the past years. He was also part of Coding club in his school and built a few robotics projects for school events. His other interests include maths, physics, numismatics and history.

Team Photo: 
<TODO>

## Project Overview
This project is our official entry for the Future Engineers category at the World Robot Olympiad 2025. Our goal is to construct a self-driving vehicle capable of navigating the track in both the Open and Obstacle Rounds. The programs are written in C++ and Python in the VSCode IDE, with the PlatformIO extension. This repository contains the programs, hardware description and design files of our model.

## Hardware Components
- __Compute:__ Raspberry Pi 5 (main computer), Raspberry Pi 2040 (Waveshare RP2040-Zero, real-time control) (Use of Arduino for initial testing)
- __Sensors:__ 1D and 2D LiDAR, IMU , rotary encoders motor, Picamera
- __Actuators:__ N20 DC gear motor (with encoders), MG996R 180° servo (steering)
- __Chassis:__ Commercially-available base, 3D-printable modifications (Links/STL files included)
- __Electronics:__ Custom peripherals board PCB for easy connections and sensor breakout

See <link> for more details about sensors on the peripherals interface board, and <link> for details about the RPi 5 and its sensors.

## Repository Structure
- [Design Files](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/Design%20Files) : Contains the Bill of Materials, 3D-printable design files and hardware. Pictures and description of the robot and components are included.
- [Initial Tests](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/Initial%20Tests) : Intial tests on various sensors to ensure accurate data collection and basic object detection algorithm.
- [hw](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/hw) : Contains schematic and PCB files for the peripherals interface board.
- [sw](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/sw) : Contains PlatformIO project for the peripherals interface board.
- [Open Round](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/Open%20Round) : It contains all the program files for the open round over various iterations and hardware setups.
- Obstacle Round : 

## System Architecture 
-	Raspberry Pi 5: It uses ROS2 and handles more complex tasks like object and colour detection, route planning, and the 2D LiDAR.
-	Raspberry Pi 2040 (Pico): Handles real-time tasks (motor PID, encoder feedback, servo control, IMU, 1D LiDAR).
-	LiDARs: Used for detecting direction, obstacles and parking area
-	IMU: Gives yaw (and other orientations) of the robot in [0,360).
-	Motor/Servos: PWM and PI controls respectively for movement using sensor data, coordinated by the RP2040.
