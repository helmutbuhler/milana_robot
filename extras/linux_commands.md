# Linux Commands
## Overview
Here are some commands that might come in handy. You can use them once you are connected to your Jetson Nano via ssh/putty.

## Basic stuff

Shutdown: `sudo shutdown now`

Reboot: `sudo reboot`

Change dir: `cd xxx`

List files: `ls`

Edit file: `nano xxx`


## List all USB devices

Run this to list all connected USB devices.
```
lsusb
```

## Show Nvidia Jetpack version
```
head -1 /etc/nv_tegra_release
sudo apt-cache show nvidia-jetpack
```

## Show CPU usage
```
jtop
```

## Change Linux bootloader commandline
```
sudo nano /boot/extlinux/extlinux.conf
```


## Check I2C open addresses
```
i2cdetect -y -r 1
```
Open addresses should be:

IMU: 0x53 0x0F 0x68

Servo: 0x40, 0x70

A2D: One of the following: 0x48  (-), 0x49  (+), 0x4A  (SDA), 0x4B  (SCL)

## SPI
List loaded spi devices:
```
ls /dev/spi*
```

Test SPI:
```
wget https://raw.githubusercontent.com/torvalds/linux/v4.9/tools/spi/spidev_test.c
gcc -o spidev_test spidev_test.c
# Place a jumper to short pins 19 & 21 on the Jetson Nano 40 pin header (for SPI1), then run spidev_test with the following command:
./spidev_test -D /dev/spidev0.0 -v -p "HelloWorld123456789abcdef"
```

If SPI is not working, check GPIO status:
```
sudo cat /sys/kernel/debug/tegra_gpio
```
The first lines should look like this:
```
Name:Bank:Port CNF OE OUT IN INT_STA INT_ENB INT_LVL
 A: 0:0 64 40 40 04 00 00 000000
 B: 0:1 00 00 00 00 00 00 000000
 C: 0:2 00 00 00 00 00 00 000000
```

## Show GPIO state
```
sudo cat /sys/kernel/debug/gpio
```


## Change I2C frequency
```
sudo nano /sys/bus/i2c/devices/i2c-1/bus_clk_rate
```

## List and kill processes
```
sudo ps -a
# kill all robot.out processes:
sudo killall -9 robot.out
# kill specific pid:
kill -9 *pid*
```


## Used diskspace / diskusage, RAM usage:
```
ncdu -x /
df -h
# ram usage:
free -h
```
## Free docker diskspace
```
sudo docker system prune -a
```

## Debugging with GDB

Attach to running process:
```
# find process id:
pgrep robot
sudo gdb -p *pid*
```

Start robot with debugger:
```
sudo gdb ~/projects/robot/bin/ARM64/Debug/robot.out
run
```

Show info about threads:
```
info threads
```

Show code in crappy GUI:
```
tui enable
```

Useful commands while debugging:

`c` - Continues running the program until the next breakpoint or error

`s` - step into function

`n` - step over into next line

`fin` - step out

`u N` - step until linenumber N

`p var` - Prints the current value of the variable "var"

`bt` - Prints a stack trace

`q` - Quits gdb

`b` - Add breakpoint on function or linenumber


## Find/Search files
Find *jpg files in home:
```
find /home -name *.jpg  2>/dev/null
```

find big files on filesystem:
```
sudo du -sx /* 2>/dev/null | sort -n
```

## Serial/UART testing
start serial console to test tts:
```
minicom -b 921600 -D /dev/ttyTHS1
minicom -b 921600 -D /dev/ttyS0
```

List UART ports:
```
dmesg | grep -i THS
```

## Kernel log
```
tail -F /var/log/kern.log
```


## Connect WIFI
Status: `nmcli d`

List wifis: `nmcli dev wifi`

List known wifis: `ls /etc/NetworkManager/system-connections`

Rescan: `sudo nmcli device wifi rescan`

Delete connection: `sudo nmcli con delete "xxx"`

Enable: `nmcli r wifi on`

Connect: `sudo nmcli d wifi connect wifi_name password xxx`

(reboot if connection fails or no wifis are listed)

Disable wifi temporarily: `ifconfig wlan0 down`

## Configure bash alias
```
nano ~/.bashrc
alias robot="sudo ~/projects/robot/bin/ARM64/Release/robot.out"
# reload .bashrc:
source ~/.bashrc
```

## UART init:
```
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
udevadm trigger
# Allow non-root users access to UART:
sudo adduser *username* dialout
sudo chmod 666 /dev/ttyS0
sudo chmod 666 /dev/ttyTHS1
```


## Make I2C speed configurable without sudo
Source is [here](https://unix.stackexchange.com/a/409780)
Create this file with the following content:
```
/etc/systemd/system/bus_clk_rate_permissions.service
```
```
[Unit]
Description=bus_clk_rate writable to everybody

[Service]
Type=oneshot
User=root
ExecStart=/bin/bash -c "/bin/chmod 666 /sys/bus/i2c/devices/i2c-1/bus_clk_rate"

[Install]
WantedBy=multi-user.target
```

Then run this:
```
sudo systemctl enable bus_clk_rate_permissions.service
sudo systemctl start bus_clk_rate_permissions.service
sudo systemctl status bus_clk_rate_permissions.service
```


## Install nvidia docker
On VM:
```
sdkmanager --cli --query interactive
```
(Select option 1 and login via nvidia account)

Select these options:
```
- select product: Jetson
- hardware configuration: Target Hardware
- select target: Jetson Nano modules
- select target operating system: Linux
- select version: JetPack 4.6.4
- get detailed options: Yes
- select Jetson SDK Components
- flash the target board: No
```
Install stuff on your Nano? Select Install
- Connect via ethernet cable
- IPv4
- Type in IP of Nano (find out on Nano via ifconfig)

