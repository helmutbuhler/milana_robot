# Milana - A Two-Wheeled Jumping Robot
## Overview
Milana is a small and relatively cheap robot on two wheels. It has 6 motors on each side:
 - A BDC 280KV motor controlling the wheel.
 - A BDC 100KV motor controlling the knee (used for jumping).
 - A 35kg servo motor controlling the hip.
 - 3 1.5kg small servo motors, controlling the arm.

The brushless motors are controlled with two [ODrives](https://odriverobotics.com/), each with custom firmware to enable some of the tricks demonstrated in the [video](todo). All motors, except for the arm motors, also have precise encoders attached.
The robot is powered by two 3Ah 3S Lipo batteries and controlled by a Jetson Nano. The Jetson Nano is running custom control software on Ubuntu with a slightly modified Linux kernel. Beside the ODrives, the Jetson Nano is connected to the following:
 - IMU sensor for 3D Orientation.
 - Servo Driver to control all servo motors.
 - A2D Converter for battery charge measurement.
 - Microphone and speaker.

## Control
The robot can be controlled externally using various methods:
 - With custom software (Control UI), running on a separate computer and connected via Wifi. This can also be used to tune and calibrate internal parameters, debug issues and simulate certain parts of the robot.
 - With a wireless XBox controller.
 - With speech recognition and a LLM. The robot can reply using speech synthesis and execute a number of commands.

 ## Abilities
 The robot can...
 - balance on its wheels while driving and changing its body posture.
 - move its "knees" so fast that it will jump a few centimeters.
 - control each arm with 3 degrees of freedom and pick up light objects with them.
 - move up to todo km/h.
 - keep balance while moving with one wheel on a small ramp.

## Code
This robot is completely Open Source: The code for the "brain" of the robot is in this repository [here](robot/). The software to control and debug the robot, [Control UI](control_ui/), is also here. There are also modules for [Automatic Speech Recognition](asr/), [Large Language Model](llm/) and [Text To Speech](tts/) included here that can optionally be used to have a conversation with the robot. Some of the code is in other repositories: The custom image for the Jetson Nano is [here](https://github.com/helmutbuhler/jetson_nano_image_milana). And the custom firmware image for the ODrives is [here](https://github.com/helmutbuhler/odrive_milana). All of these links point to a readme with more information about each component.

## Building
The CAD files and the meshes that need to be printed with a 3D printer are [here](CAD/). Other things that you need to buy is listed in the [Bill of Materials](extras/BOM.ods) (the total is about 1200€). More information about each part is in the comments of the code that controls it.

This is not a project for beginners. If you intend to make the robot, some experience in robotics and coding is recommended. I also don't provide any warranty of any kind. Use all of this at your own risk.

