#!/usr/bin/env python
'''
Calvin Li
April 2017


Module for processing the sorted data from auv_readSensor.py
After the //data// is processed by finding how the vehicle should orient itself in order to reach it's destination,
commands are then passed to the motors.
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class process(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(process, self).__init__(mpstate, "process", "")
        self.status_callcount = 0
        self.boredom_interval = 2 #seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.process_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('process', self.cmd_process, "process module", ['status','set (LOGSETTING)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: process <status|set>"

    def cmd_process(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.process_settings.command(args[1:])
        else:
            print self.usage()

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    def boredom_message(self):
        if self.process_settings.verbose:
            return ("Waiting for telemetry information from I2C port. Is the readSensor module working properly?")
        return ("Waiting for telemetry information.")

    def idle_task(self):
        '''
        Called rapidly by mavproxy.
        Unceasingly processes the telemetry data inside the results.txt file.
        Calculates the necessary roll, pitch, yaw, direction, bearing, etc...
        '''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                            message)

            #read the results file for data
            data = open("~/results.txt", "r")

            #use readline(line number) on the line containing the information desired
            #DATA LINE NUMBERS
            #Roll-1 Pitch-2 Yaw-3 DesRoll-4 DesPitch-5 DesYaw-6
            roll = data.readline(1)
            pitch = data.readline(2)
            yaw = data.readline(3)

            #Calculate the current roll, pitch, yaw, etc for the AUV
            #Pitch and roll should be neutral, yaw should be pointing towards destination

             

        #endif

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1

def init(mpstate):
    '''initialise module'''
    return process(mpstate)
