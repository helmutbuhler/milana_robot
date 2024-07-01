# Robot application
## Overview
This is the application that runs on the Jetson Nano inside of the robot and controls pretty much all of the components.

The components correspond the the .cpp files here:
- asr_client
- balance_control
- battery
- body_acc
- command
- foot_control
- ik
- imu
- joystick
- leg_control
- main
- pin
- server
- servo
- statistics
- tts_client

More details about them is in the comments in the source.

The application has a main loop that runs at about 60Hz and updates all the components in each iteration. It's defined in `main.cpp`. You will notice there that all the components are very independent of each other. It's easy to disable them individually. This is very helpful for testing things. Initially you will want to comment them all out, except for the server, and test network connection with Control UI. After that works you can uncomment them step by step.

## Jetson Nano Configuration
You need to install the custom [Milana Image](todo) on your Jetson Nano so that both UART ports work (we need those to communicate with the ODrives). It's also required to make the SPI and I2C ports more reliable. Detailed instructions on how to install it are there.

Once you have that installed and established SSH via a WIFI connection, we need to configure it a bit more:

We need some packages: (We need python so that ODriveTool will work)
```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update && sudo apt-get install gcc-arm-embedded openocd git-lfs python3 python3-yaml python3-jinja2 python3-jsonschema meson portaudio19-dev python3-pip libopenblas-dev minicom nano

pip3 install PyYAML Jinja2 jsonschema requests pyusb intelhex appdirs ipython pyserial
sudo -H pip3 install -U jetson-stats
```

We also need to enable the SPI pins (for communication with the IMU) and the I2S pins (for communication with the MAX98357A board that controls the speaker):

```
sudo /opt/nvidia/jetson-io/jetson-io.py
Configure 40 pin
Configure header pins manually
enable i2s4 and spi1
Save pin changes
Save without reboot
```

And we need to initialize the audio driver once:
```
alsactl init
```

## Build
Building this application can be done directly on the robot with CMake, or indirectly through Visual Studio.

### CMake
Clone this repository on your Jetson Nano, cd into it and do this:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
If you get errors involving ASR and Cuda, comment that project out of the `CMakeLists.txt` file. We don't need the ASR Server on the Jetson Nano.

Once it's compiled you can start it with:
```
sudo ./robot
```
You can also test compilation on a Desktop Linux or in WSL. But it will obviously not run there.

### Visual Studio
To build from a Windows machine, you need VS2019 or up. Clone this repository on your Windows machine and open `milana_robot.sln`. The project `robot` in there is a "Raspberry" project which is built remotely on our Jetson Nano. You need to install the `Linux Development with C++ workload` workload in Visual Studio for this to work. I found it convenient to build this way because you can develop on a real PC and it only needs to upload the changed files. See [here](https://devblogs.microsoft.com/cppblog/linux-development-with-c-in-visual-studio/) for more information.

If you get build errors, copy this folder `3rdparty/eigen` from the repo into: `~/projects/3rdparty/eigen` (I didn't figure out how to do this automatially).

After a successful build, you need to start the application through ssh:
```
sudo ~/projects/robot/bin/ARM64/Release/robot.out
```
(Starting and Debugging from Visual Studio won't work because the application needs sudo rights)

You can optionally define an alias, so you just need to type `robot`:
```
nano ~/.bashrc
# append the following:
alias robot="sudo ~/projects/robot/bin/ARM64/Release/robot.out"
# reload .bashrc
source ~/.bashrc
```
