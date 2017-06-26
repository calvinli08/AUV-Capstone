# AUV-Capstone

# Intro
This is an implementation of the adaptive sampling algorithm proposed in [Pompili et al. (2012)](http://www.ece.rutgers.edu/~pompili/paper/05_paper29.pdf) on the BlueROV2 from BlueRobotics.
The files included in this repository will prepare a BlueROV2 for autonomous navigation, via a MAVProxy module designed to be operated from the CLI. Before taking your AUV out for a dive, please carefully read the instructions below.

## Materials

This software requires a BlueROV2 for it to be loaded and used properly, although with minor modifications it can be used to automate other small submersibles.
If a BlueROV2 has not yet been purchased and assembled according to the manufacturer's instructions, please go to
[BlueRobotics.com](http://bluerobotics.com/store/rov/bluerov2/) to purchase and assemble. The Advanced Electronics package with a Raspberry Pi 3 computer is required. Purchasing a set of spare parts is also highly recommended.
To facilitate the software setup, please refrain from sealing the electronics tube before setup is complete.
As assembly is progressing, make note of the direction of the propellers (cw, ccw) as this will affect movement later on.

All the software that this code depends on is open-source:
[Mavlink](http://qgroundcontrol.org/mavlink/start)
[MAVProxy](http://ardupilot.github.io/MAVProxy/html/getting_started/index.html)
[QGroundControl](http://qgroundcontrol.com/)
[ArduSub](https://www.ardusub.com/)
[Python2.7](https://www.python.org/downloads/)

## Work Environment

There are two separate fronts that need to be understood before automating the ROV. Firstly, there is the hardware.
Please become familiar with the navigational and electrical engineering concepts listed below in the order presented:

### Hardware Concepts
### Required Software

## Loading this Module

## Pool Tests

__Caution__:

##

http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial

The implementation of our autonomous algorithm for the BlueROV2.
MATLAB based simulation files are in the SIMULATIONS folder.
MAVProxy dependent Python implementations of the algorithm proposed in Pompili et al. are in DEPLOYMENT.
Requires MAVProxy.
