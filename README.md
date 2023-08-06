# KBOT Drive
## Install Arduino IDE
1. Download [Arduino IDE](https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz)
```sh
$ cd ~/Download
$ tar xvf arduino-1.8.19-linux64.tar.xz
$ cd arduino-1.8.19-linux64
$ sudo ./install.sh
```
2. Add Preferences in Arduino IDE https://dl.espressif.com/dl/package_esp32_index.json
3. Setup Python and Serial
```sh
$ sudo apt install python-is-python3
$ sudo apt install python3-serial
```
4. Setup USB permission
```sh
$ sudo nano /etc/udev/rules.d/50-myusb.rules

SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="users", MODE="0666"
```

## Install rosserial
1. Arduino IDE Setup (credit: [rosserial_arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup))
```sh
$ sudo apt-get install ros-noetic-rosserial-arduino
$ sudo apt-get install ros-noetic-rosserial
```
2. Install library
```sh
$ cd Arduino/libraries
$ rosrun rosserial_arduino make_libraries.py .
```
