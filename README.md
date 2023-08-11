# KBOT Drive
## Install Arduino CLI
1. Download and install arduino-cli
```sh
$ cd ~/.
$ curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
```
2. Adding Arduino CLI path
```
$ nano ~/.bashrc
export PATH=$PATH:/home/kbot/bin

$ sudo reboot now
```
3. Create a configuration file and install ESP32 board
```
$ cd ~/.
$ arduino-cli config init
$ nano ~/.arduino15/arduino-cli.yaml
board_manager:
  additional_urls:
    - https://dl.espressif.com/dl/package_esp32_index.json

$ arduino-cli core update-index
$ arduino-cli core install esp32:esp32@1.0.6
```
4. Setup Python and Serial
```sh
$ sudo apt install python-is-python3
$ sudo apt install python3-serial
```
5. Setup USB permission
```sh
$ sudo nano /etc/udev/rules.d/50-myusb.rules
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="users", MODE="0777"
```

## Install rosserial
1. Arduino IDE Setup (credit: [rosserial_arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup))
```sh
$ sudo apt install ros-noetic-rosserial-arduino
$ sudo apt install ros-noetic-rosserial
```
2. Install library
```sh
$ mkdir -p ~/Arduino/libraries
$ cd ~/Arduino/libraries
$ rosrun rosserial_arduino make_libraries.py .
```
3. Adding custom message(Optional:[kbot_msgs](https://github.com/aims-lab-kmitl/kbot_msgs.git))
```
$ rosrun rosserial_client make_library.py ~/Arduino/libraries kbot_msgs
```
## Download kbot_drive.ino
```
$ cd ~/.
$ git clone https://github.com/aims-lab-kmitl/kbot_drive.git
```

## Compile Arduino-CLI
```
$ cd ~/kbot_drive
$ arduino-cli compile -b esp32:esp32:esp32 .
```
## Uploda Arduino-CLI
```
$ cd ~/kbot_drive
$ arduino-cli upload -p /dev/ttyUSB0 -b esp32:esp32:esp32 .
```
