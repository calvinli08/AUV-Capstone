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
* [Linux distro (we used Ubuntu 14 LTS)](http://mirror.pnl.gov/releases/)
* [Mavlink](http://qgroundcontrol.org/mavlink/start)
* [MAVProxy](http://ardupilot.github.io/MAVProxy/html/getting_started/index.html)
* [QGroundControl](http://qgroundcontrol.com/)
* [ArduSub](https://www.ardusub.com/)
* [Python2.7](https://www.python.org/downloads/)
* [ArduSub custom Raspbian image](https://www.ardusub.com/resources/downloads.html#raspberry-pi-images)
* SSH client for smart phones


## Work Environment

  There are two separate fronts that need to be understood before automating the ROV.

### Hardware
  Underwater communication primarily relies on slow moving acoustic waves. As such, there are no GPS or WiFi signals underwater. This limits how deep the AUV can dive before it must become fully autonomous (in the absence of the tether). Any manual control of the AUV is then only possible when it is on the surface. During the course of testing, we have discovered that communicating with the on-board RPi is best done with a smart phone, as smart phones tend to seek WiFi connections more aggressively than laptops do, making them ideal for keeping a strong communication with the AUV when it is in the water and at a moderate distance (about 4 feet).

  __Warning__: _The thrusters can be more powerful than expected. If you have watched the YouTube video demos of the BlueROV2, the force at which the vehicle propels itself is not immediately apparent. Before attempting any movement, make sure that the intended direction and PWM values are all known and inputted correctly; in addition, be sure to have a failsafe. Failure to heed this warning will leave you wrestling with the AUV to try and send it a stop command (if you can catch it that is)._

  The PixHawk autopilot is shipped without a GPS so one must be purchased independently. We recommend searching on HobbyKing.

  The BlueROV2 has six degrees of freedom (forward, backward, left, right, up, and down) and six high-powered (and do they mean high powered) thrusters to provide said degrees. Notice that there are several channels leaving the PixHawk, connected to the ESC's (Electronic Speed Control). These channels correspond, not to the individual thrusters, but to the different degrees of freedom (i.e. activating channel 4 does not individually spin propeller 4). Individual speed of each degree of freedom is controlled by a [Pulse Width Modulation](https://en.wikipedia.org/wiki/Pulse-width_modulation) value; the safe range is from 1300 - 1700.

  If at times the AUV does not move in the intended direction, consider checking which way the thrusters propel water. In the event that the propulsion is not correct, the orientation of the thrusters may need to be changed in QGroundControl, in the Parameters section.

### Software
  Mavlink is a communication protocol that is used to translate messages from a computer to the PixHawk. A Python implementation called pymavutil.py has functions for processing incoming mavlink messages, and sending mavlink messages to the PixHawk. Examples of mavlink packets are instructions to move the vehicle in a direction, GPS information, battery information, GPS waypoints, etc. MAVProxy, which is a CLI that can be used to control the BlueROV2, is essentially a wrapper for pymavutil.py.  

## Software Setup
  Download and install the software listed under [Materials](https://github.com/Snowrries/AUV-Capstone/tree/master#materials).

  Clone this git repository and copy the mavproxy_auto folder (the folder itself as well) into the MAVProxy/modules directory (by default it is in /usr/local/lib/python2.7/dist-packages/MAVProxy/modules)



## Pool Tests

__Caution__:
