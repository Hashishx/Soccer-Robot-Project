![Picture1](https://github.com/user-attachments/assets/529aa5c5-8562-4a32-8c40-b589a69a1033)

Autonomous Robot with Mobile Camera Integration
This repository contains the code and documentation for an autonomous robot system. The robot navigates towards a red object using a mobile camera, which communicates with a server to process the object’s position and send movement commands to the robot via an ESP8266 module.

Project Overview
The system consists of three main components:

Mobile Camera: Captures video and streams it to the server.
Server: Processes the video feed to locate the red object and sends movement commands based on its position.
Robot: Receives commands via the ESP8266 module to control its DC motors for movement.
Features
Autonomous navigation toward a red object.
Real-time object detection and movement.
Wireless communication between server and robot using UDP protocol.

Modular design with separate responsibilities for the camera, server, and robot.
System Architecture
Camera:

The mobile camera streams video to the server.
The server uses image processing techniques (OpenCV) to detect the position of the red object.
Server:

Processes the video feed to determine the object's position.
Sends movement commands ("Forward", "Backward", "Right", "Left", "Stop") to the robot based on the object’s relative position.
Robot:

The ESP8266 receives commands from the server over UDP.
Controls two DC motors to move the robot forward, backward, left, or right.
Stops the motors when no command is received.

How It Works
The camera captures a live video feed and sends it to the server.
The server processes the video in real time to detect the red object's position using image processing.
Based on the red object's position, the server determines the robot's movement and sends commands via UDP.
The ESP8266 receives the commands and drives the robot's motors accordingly.
