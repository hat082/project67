# Raspberry Pi Robot Control and Image Processing Program
Construction Project Week 6, 7, 8

## Introduction
The Raspberry Pi Robot Control and Image Processing Program is an advanced system designed to manage the operations of a robot using a Raspberry Pi board. The program is equipped with capabilities for line following, color detection, template matching, and task execution, all of which are facilitated by image processing and hardware control functionalities.

## Prerequisites
Before delving into the specifics of the program, ensure you have the following:
- A Raspberry Pi board
- Compatible camera module for Raspberry Pi
- WiringPi and softPwm compatible actuators (LEDs, servo motors, etc.)
- OpenCV library for image processing
- Appropriate development environment set up for C++ programming

## Program Overview
The program is structured into several sections, each serving a distinct purpose in the overall operation of the robot. Here is a breakdown of the key components:

### Header Files
- **algorithm**: For sorting functionality.
- **chrono**: For time-related operations.
- **dirent.h**: For directory entry operations.
- **exception**: For exception handling.
- **locale**: For internationalization support.
- **opencv2/opencv.hpp**: The main OpenCV library for image processing.
- **softPwm.h** and **wiringPi.h**: For controlling GPIO pins.
- **stdio.h**: Standard input/output library.
- **vector**: For dynamic arrays.

### Defines and Global Variables
- Pin definitions for PWM and LEDs.
- Global variables for storing frame data, error values, and color thresholds.

### Helper Functions
- `loadTemplates()`: Loads image templates for object recognition.
- `moveCamera()`: Controls the camera's position using a servo motor.
- `setup()`: Initializes the hardware and sets up the initial state.
- `detectContours()`: Identifies contours in the image for object detection.
- `preprocessFrame()`: Preprocesses the image for better analysis.
- `imgcap()`: Captures an image from the camera and extracts a region of interest.
- `errorCalc()`: Calculates the error for line following.
- `offsetCalc()`: Calculates the offset for motor control using a PID controller.
- `sendCmd()`: Sends commands to control the robot's speed.
- `existPink()`: Checks for the presence of pink, triggering a specific state.
- `reset()`: Resets the system to its initial state.
- `templateMatching()`: Matches templates to identify objects.
- `performTask()`: Executes the task associated with the detected object.

### Main Function
- Sets up the robot and enters a loop where it continuously captures images, processes them, and performs tasks based on the detected objects and colors.

## Usage
```
g++ -o robot_control main.cpp `pkg-config --cflags --libs opencv4`
./robot_control
```

## License
This program is distributed under the MIT License, allowing for personal and commercial use, modification, and distribution.

## Contributing
Contributions to the program are welcome and can be submitted via pull requests to the official repository.

