#!/usr/bin/env python

'''Module that continously updates navigational parameters (attitude)'''
import os
import os.path
import sys
from pymavlink import mavutil as mav
import errno
import time
import numpy

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class NavigationModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(NavigationModule, self).__init__(mpstate, "nav", "Update attitude", public=True)

        '''Attitude'''
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.relative_alt = 0
        self.vx = 0
        self.vy = 1
        self.vz = 1
        self.hdg = 0

        self.add_command('nav', self.cmd_nav, 'Start nav daemon', ['<start|stop>'])

    def cmd_nav(self, args):
        if len(args) != 1:
            return "Usage: nav <start|stop>"
        elif args[0] == 'start':
            self.idle_task()
        elif args[0] == 'stop':
            sleep(120)
        else:
            return "Usage: nav <start|stop>"

    '''Public function for use by other modules to grab real time attitude information'''
    def get_attitude(self):
        return [self.lat, self.lon, self.alt, self.relative_alt, self.vx, self.vy, self.vz, self.hdg]

    def gps_update(self, GLOBAL_POSITION_INT):
        '''update gps readings'''
        self.lat = GLOBAL_POSITION_INT.lat
        self.lon = GLOBAL_POSITION_INT.lon
        self.alt = GLOBAL_POSITION_INT.alt
        self.relative_alt = GLOBAL_POSITION_INT.relative_alt
        self.vx = GLOBAL_POSITION_INT.vx
        self.vy = GLOBAL_POSITION_INT.vy
        self.vz = GLOBAL_POSITION_INT.vz
        self.hdg = GLOBAL_POSITION_INT.hdg

    '''sleep'''
    def idle_task(self):
        # wait for mavlink packets
        print self.hdg/100

    def mavlink_packet(self, m):
        mtype = m.get_type()
        if mtype == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
             self.gps_update(m)


def init(mpstate):
    '''initialise module instance'''
    return NavigationModule(mpstate)
