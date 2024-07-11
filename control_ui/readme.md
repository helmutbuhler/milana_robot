# Overview

control_ui is an application that connects to the robot via WIFI and gives you a lot of features:
- Control the robot via keyboard (arrow keys to move, P, L, O, K to move joints)
- Monitor about 300 variables received by the robot. Displayed as plots.
- Control lots of variables on the robot (tune pid controllers etc.)
- Display usb webcam data and sync it to the data received from the robot.
- Display sensor state in 2D and 3D models of the robot.

When not connected to the robot, this can also be used to run simulations of the robot
(mainly of the movement of the wheels) and to debug IK stuff.
It can also be used to load capture data that was saved to a file earlier.

Note that the robot can also work when control_ui is not connected to it.

[Here](https://github.com/helmutbuhler/odrive_control_ui) is a repository with a trimmed down version of this application. This might be a better starting point for new projects.

## Build
### Windows
To build on Windows, you need VS2019 or up. Clone this repository and open `milana_robot.sln`. It should build out of the box.

### Linux
To build on Linux, first we need some packages:
```
sudo apt install cmake libglfw3-dev libglm-dev libusb-1.0-0-dev libasound2-dev build-essential gnome-terminal
```

Next clone the repository, cd into it and do this:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
If you get errors involving ASR and Cuda, comment that project out of the `CMakeLists.txt` file. See in the `asr` folder on how to compile that part.

Once it's compiled you can start Control UI with:
```
./control_ui
```
