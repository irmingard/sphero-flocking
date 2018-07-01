# Requirements

* Ubuntu 12.04
* BlueTooth
* One or more Sphero 2.0 robots

## Installing Ubuntu 12.04

* Download Ubuntu 12.04 image
(http://releases.ubuntu.com/12.04/ubuntu-12.04.5-desktop-amd64.iso)

* Create a bootable flash drive and put Ubuntu image on it
(e.g. use Win32 Disk Imager: https://sourceforge.net/projects/win32diskimager/)

* Boot from flash drive

* Install Ubuntu (wipe Windows or install alongside it)

## Software

* Required: NetLogo for Linux
(https://ccl.northwestern.edu/netlogo/5.3.1/)

* Recommended IDE: PyCharm Community for Linux (https://www.jetbrains.com/pycharm/download/#section=linux)

### Middleware: ROS

* Follow the instructions on (http://wiki.ros.org/groovy/Installation/Ubuntu) for Ubuntu 12.04 (Precise) and do the Desktop-Full Install

### Sphero driver

* Download Sphero ROS driver
(https://github.com/mmwise/sphero_ros/archive/groovy-devel.zip)

* Extract ZIP

* Open Terminal

* Enter in Terminal (adapt path if ZIP was downloaded to somewhere else):


    cd Downloads/spheros_ros-groovy-devel/sphero_driver
    python setup.py build
    sudo python setup.py install

## Prepare Spheros

* Fully charge Spheros
* Download the Sphero Android app (https://play.google.com/store/apps/details?id=orbotix.sphero&hl=en) and update the firmware for all Spheros
* Pair your Spheros via Bluetooth and write down their MAC addresses
* Update the MAC addresses values in the `bidirectional.py` to match your own MAC addresses

## Prepare environment

* The standard environment is 250 x 250 cm. You will also have to measure out the robot's starting positions that are set in the NetLogo model.

# Start model run

* Open `bidirectional.nlogo` in NetLogo
* Open `bidirectional.py` in PyCharm
* Run `bidirectional.py`
* Follow Python console instructions

# Troubleshooting

* Stop everything, turn Bluetooth off and then back on