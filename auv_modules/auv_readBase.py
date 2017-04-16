#!/usr/bin/env python
'''
Calvin Li
April 2017


This is the module used for reading input data from the common database used in the demo. Data is read in and
sorted and is afterwards passed to auv_process.py for calculating how motors should run.
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


class read(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""

        #Interval of small as possible. ms even.
        super(read, self).__init__(mpstate, "read", "")
        self.status_callcount = 0
        self.boredom_interval = 1 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        #True for debugging purposes
        self.verbose = True

        self.read_settings = mp_settings.MPSettings(
            [ ('verbose', bool, True),
          ])
        self.add_command('read', self.readSensor, "Read data from the Pixhawk's I2C port", ['status','set (LOGSETTING)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: read <status|set>"

    def cmd_read(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.read_settings.command(args[1:])
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
        if self.read_settings.verbose:
            return ("Accessing data from Pixhawk I2C port")
        return ("Reading I2C on port" + #port number)

    def read(self):
        '''
        Called rapidly by mavproxy
        The main component of this module. A read function that is called rapily and automatically in order to ensure
        a real time conception of the AUV's navigational information by the Raspberry Pi.
        '''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                            message)

            #Read the I2C port data by accessing a tlog file
            tlog = open("~/mav.tlog", "r")

            #find out on which line the data resides
            tlog.readline() #parse the format of the file

            #write the data to another file with simple format after reading it
            #Format is to write data such as height, velocity, etc in a set order, with a new line for each.
            #This way, in order to access a certain piece of data, just access that line using readline()
            results = open("~/results.txt", "w")

            #write the data; don't forget an EOL \n character!
            #DATA LINE NUMBERS
            #Roll-1 Pitch-2 Yaw-3 DesRoll-4 DesPitch-5 DesYaw-6
            #
            results.write("\n")


            #close file
            results.close()

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
    return read(mpstate)
