# Ctrl-Alt-Defeat-WRO-Future-Engineers-2025

Welcome to the repository for our entry in the WRO Future Engineers 2025 competition!
We are a team of 2 aspiring engineers building an autonomous robotic car using Raspberry Pi 5 & RP2040, equipped with various sensors and custom designs.

Team Photo: 
![Team Photo](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/blob/main/repo-assets/raw-photos/DSCF1097.JPG)

## About the Team
Adbhut Patil: TODO: expand 10th standard, interested in electronics, programming, aviation

Pranav Kiran Rajarathna: Pranav is currently sudying in the 11th grade(PCMC combination) and has been interested in Robotics for several years . He has participated in WRO in other categories in the past years. He was also part of Coding club in his school and built a few robotics projects for school events. His other interests include maths, physics, numismatics and history.

## Project Overview
This project is our official entry for the Future Engineers category at the World Robot Olympiad 2025. Our goal is to construct a self-driving vehicle capable of navigating the track in both the Open and Obstacle Rounds. The programs are written in C++ and Python in the VSCode IDE, with the PlatformIO extension. ROS is used with the Raspberry Pi for object detection and navigation. This repository contains the programs, hardware description and design files of our model.

## Hardware Components
- __Compute:__ Raspberry Pi 5 (main computer), Raspberry Pi 2040 (Waveshare RP2040-Zero, real-time control) (Use of Arduino for initial testing)
- __Sensors:__ 1D and 2D LiDAR, IMU , rotary encoders motor, Picamera
- __Actuators:__ N20 geared brushed DC motor with encoder, MG996R 45° servo
- __Chassis:__ Commercially-available base, 3D-printable modifications (Links/STL files included)
- __Electronics:__ Custom peripherals interface PCB for reliable connections to sensors

See [this section](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025?tab=readme-ov-file#peripherals-interface-board) for more details about sensors on the peripherals interface board, and <link> for details about the RPi 5 and its sensors.

## Robot Assembly

It will be helpful to refer to the pictures of the completed model for the following steps.
1. Add the front wheels and steering link to the robot
2. Attach the servo motor and connect it to the steering link
3. Print all design files
4. Attach the N20 motor onto the rear drive with brackets. Fit a gear onto the motor.
5. Cut the rear axle to the appropriate length (around 96mm) and attach the rear axle with wheels and gear onto the rear drive.
6. Mount the rear drive onto the chassis.
7. Attach 6 pillar screws of length 26mm in the back and 2 of length 22mm in the front (adjacent to the servo)
8. Attach the middle plate at the 6 rear points, but not at the front. The peripherals board may be screwed on using 4 pillar screws.
9. Attach the TFLuna LiDARs to the front LiDAR holder section, followed by the RPLiDAR (remember to connect the USB adapter) with 22/24 mm screw pillars. Screw in the front points.
10. Attach the LiDAR holder section to the front of the chassis.
11. Attach the additional support section for the LiDAR holder to the 2 front middle plate points and 2 rear RPLidar points. A longer screw (at least 10mm) is needed for this.
12. Attach the top-most Raspberry Pi platform to the rear with 18 mm screw pillars.
13. Attach the Raspberry Pi with the UPS Hat to the top.
14. Make all necessary connections.

## Repository Structure
- [Design Files](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/Design%20Files) : Contains the Bill of Materials, 3D-printable design files and hardware. Pictures and description of the robot and components are included.
- [Initial Tests](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/Initial%20Tests) : Intial tests on various sensors to ensure accurate data collection and basic object detection algorithm.
- [hw](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/hw) : Contains schematic and PCB files for the peripherals interface board.
- [sw](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/sw) : Contains PlatformIO project for the peripherals interface board.
- [Open Round](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/Open%20Round) : It contains all the program files for the open round over various iterations and hardware setups.
- Obstacle Round : 
- [test-data-recordings/open-round](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/test-data-recordings/open-round) : It contains data logs from various open round tests

## System Architecture 
-	Raspberry Pi 5: It uses ROS2 and handles more complex tasks like object and colour detection, route planning, and the 2D LiDAR.
-	Raspberry Pi 2040 (Pico): Handles real-time tasks (motor PID, encoder feedback, servo control, IMU, 1D LiDAR).
-	LiDARs: Used for detecting direction, obstacles and parking area
-	IMU: Gives yaw (and other orientations) of the robot in [0,360).
-	Motor/Servos: PWM and PI controls respectively for movement using sensor data, coordinated by the RP2040.

### Mobility

- Configuration: Front-wheel steering (MG996R) with a single axle DC rear drive motor (N20).
- Chassis: A commercially available metal chassis base with custom 3D-printed additions.
- Control: PWM-based speed control, servo-based steering.
- Turning Radius: Optimized for narrow WRO track corners, and is 32-33 cm.
- Build Choice Reasoning: Offers realistic car-like dynamics, ideal for FE challenge simulation.

### Power
  TODO: Inaccurate, update once power system is finalised
- The RP2040 system runs on a pair of 3.7V 18650 batteries outputting 7.4V for the IMU, N20 motor, servo and 1D-LiDARs.
- The Raspberry Pi uses a pair of 3.6V 21700 batteries with a UPS Hat for power
- This way both the sets of batteries can be neatly fit in the spaces in the model at the base and below the RPi.

### Sensors

- __TFLuna LiDAR:__ These are 1D-LiDARs that are used to measure distances to the front, left and right of the robot. They are connected to the RP2040.
- __BNO055:__ This is the IMU we are using for orientation due to its high accuracy and simplicity of use. It is connected to the RP2040.
- __nRF24L01:__ This is the wireless module used for wireless communication during testing and debugging with the RP2040. Another custom module (telemetry board) is made for receiving the data.
- __Motor encoders:__ These give ticks each time they are trigerred by rotation. It is connected to the RP2040 and used for distance calculations.
- __RPLidar A1:__ It is a 2D-LiDAR connected to the Raspberry Pi 5/4B for mapping, localisation and obstacle detection.
- __Raspberry Pi Camera Module 3:__ It is used with the Raspberry Pi 5/4B for recognising the colour of an obstacle once detected with the RPLidar

### Obstacle Management

The obstacle is initially detected by the Raspberry Pi using data from the RPLidar. Next, the colour of the obstacle is checked in the region of interest using the PiCamera using OpenCV. ROS is used for mapping and localisation, and the navigation of the robot is carried out with communication between the Raspberry Pi and RP2040 (Red --> Right and Green --> Left). 

## Peripherals Interface Board
![Peripherals Interface Board - Top](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/blob/main/repo-assets/edited-photos/periph-board-pcb-top.jpg)

The peripherals interface board goes between the Raspberry Pi and vehicle hardware. The board talks to three 1d TFLuna LiDARs, a 9-axis BNO055 IMU, the motor encoder, the drive motor, and the steering servo. It talks to the Pi over USB.

### Hardware
The board is a two-layer, 1.6 mm thick PCB designed in [KiCad](https://kicad.org) and manufactured by [Robu](https://robu.in).
TODO: Expand

### Software
The interface board was programmed in C++, with the C++ SDK from Raspberry Pi for the RP2040 chip. Code was flashed using the PlatformIO extension for the VSCode editor.

The codebase is designed to be modular, with easily swappable drivers for hardware abstraction. Algorithms do not need to know about specific hardware protocols or constraints

`main.cpp` loads in drivers, which implement virtual interface classes. `main.cpp` then initialises everything, and in a loop, gets data from the `SensoorManager`, passes that data to our drive algorithm, and outputs it to the `TargetController`, which commands the motor and steering servo.
Drivers are in `/src/drivers`, Interfaces are in `/src/interfaces/`, and utilities are in `/src/utils/`.

#### Code Development Standards
To ensure easily testable, upgradeable, and clean code, a few standards were adhered to when writing code for the interface board.

- `main.cpp` must be as clean as possible.
  - Logic and code flow must be easy to understand.
- All hardware-specific things should be done inside a driver.
- Drivers must take in a standard unit as their input.
  - Drivers must not take in [magic numbers](https://en.wikipedia.org/wiki/Magic_number_(programming)).

TODO: Finish writing this

#### Drivers
> [!NOTE]
> Hardware revision 1 drivers are incomplete, as the peripherals baord codebase was partly written when the switch from the prototype interface and the PCB occured, and drivers from that point on were only for the PCB (hardware revision 2).
A list of implemented drivers:
- `hwrev2_imu` : Implements `ISensor`. Driver for the BNO055. Gets orientation data processed by the BNO055's internal microcontroller.
- `hwrev2_lidar` : Implements `ISensor`. Driver for the TFLuna LiDARs. Returns a [0, 259] of distance data. The three LiDARs are mapped to `0`, `90`, and `270`. (This is a remnant of the original plan of connecting the RPLidar to the peripherals board, instead of to the Pi.)
- `hwrev2_motor_driver` : Implements `IMotorDriver`. Driver for the TB6612FNG dual-channel H-bridge motor driver. The driver only supports driving a single channel on the T66612FNG.
- `hwrev2_rf24_communication` :  Implements `ICommunication`. Driver for the nRF24L01+ radio used during development. Transmits vehicle data and gets commands from another nRF24L01+, connected to a computer.
- `hwrev2_single_lidar_open_round` : Implements `IDriveAlgorithm`. A port of our open round test algorithm. The name is misleading, the algorithm uses all three TFLunas.
- `hwrev2_steering_driver` : Implements `ISteeringDriver`. Driver for the MG996 steering servo. Takes in a degree input and coverts it to a 0-180 command with a conversion constant.
- `hwrev2_target_control` : Implements `ITargetControl`. Takes in a `VehicleCommand`, and either passes it directly through to the `ISteeringDriver` and `IMotorDriver`, or passes the command through a PID controller first.
- `hwrev2_uart_logger` :  Implements `ILogger`, Takes in a sender, message type, and a message string
- `hwrev2_vehicle_speed` : Implements `ISensor`. Gets vehicle distance from the motor encoder. Calculates speed using time between pulses.


#### Interfaces

Interfaces are virtual classes which define certain functions. Drivers implement these classes. Code calls functions defined in the interface virtual class.

- `IDriveAlgorithm` : Takes in a `VehicleData` object, and returns a `VehicleCommand` object.
- `ILogger` : Constructs a message and prints it out on a UART bus. Takes in a sender string, a log type (information, warning, error) and the message.
- `IMotorDriver` : Takes in a speed value and outputs to a motor driver.
- `ISensor` :  A generic sensor object. Returns a vector of `SensorData` objects.
- `ISteeringDriver` :  Takes in a wheel steering angle, outputs to a steering mechanism.
- `ITargetControl`: Takes in a `VehicleCommand`, outputs to a motor and steering driver.

#### Managers

Managers are classes that handle certain aspects of the vehicle. Certain structs have their header files in the `/src/managers` folder, even though they are not classes.

- `VehicleConfig` :  A struct containing information about the vehicle. A `VehicleConfig` struct is passed to all drivers asnd managers.
- `SensorData` :  A struct containing fields for all data that can be collected by the vehicle, and an enum which tells the caller what fields are being used.
- `SensorManager` :  A class which takes in an arbitrary number of sensors, collects data from them, and processes them into a `VehicleData` struct.
- `SensorStatus` :  A (currently unused) enumeration which defines keys for states a sensor can be in. 
- `Vec3f` :  A struct of three double-precisionn floating point numbers, used to represent three-axis values.
- `VehicleCommand` :  A struct defining a target speed and yaw value for the vehicle. Returned by a drive algorithm, passed to a `ITargetController`.
- `VehicleData` :  A struct containing processed vehicle data. Returned by a `SensorManager`.

### Utilities

- `SchedulerTask` : A class which calls a function periodically.
- `Schedular` : A class which takes in an arbitrary number of `SchedulerTask`s, and updates them.

## Open Round

Refer to `sw/peripherals-board/src/drivers/hwrev2/hwrev2_single_lidar_open_round.cpp and .hpp` files for program.

### Header File

Public includes the logger (for debugging), vehicle command (for driving) and sets direct control of servo (A target yaw value is generally given). Private includes setup of variables for driving, turning, servo, IMU, 1D LiDARs and IMU-based straight follower. 

### Implementation File

#### Initialisation
- Sets up logging
- Sets initial speed and steering

#### Turn Detection & Execution:
- Uses front LiDAR to detect when to start a turn (if obstacle is close).
- Decides turn direction (left/right) based on side LiDARs (First turn only)
- Turns are started considering factors of distance available, yaw, current section and distance travelled. 
- Adjusts steering (servo position) during turn based on yaw and for greater smoothness.
- After each turn, resets variables and prepares for the next segment. Target yaw and threshold distance may be altered for better accuracy and counter drifting.
- The idea to stick to the outer wall, so even with randomisation, the extended inner wall is not hit

#### Straight Movement:
- When not turning, uses gyro (yaw) to follow a straight path.
- Applies proportional-integral correction to steering for accurate straight-line travel.

#### Turn Direction Logic:
- Chooses clockwise or anticlockwise based on which side has more open space (LiDAR difference).
- Sets different thresholds for smoother turns depending on direction.

#### Stopping 
- Counts turns; after 12 turns (3 rounds), slows down and stops at the start section.

### Flow
1. Start moving straight.
2. Decide turn direction using side LiDARs. (For first turn only)
3. Detect distance to wall ahead with front LiDAR.
4. Execute turn when condition are met, adjust steering and target yaw.
5. After turn, resume straight movement with PI control.
6. Repeat for 3 rounds (12 turns).
7. After final round, slow down and stop in the start section.
