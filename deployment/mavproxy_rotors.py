#!/usr/bin/env python

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules import mavproxy_wp
from MAVProxy.modules import mavproxy_rc

class RotorsModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RotorsModule, self).__init__(mpstate, "rotor", "Rotor control", public = True)

        self.motor_event_complete = None
        self.wp_manager = mp_waypoint.WPManager(self.master, self.target_system, self.target_component)
        self.rc_manager = mp_rc.RCManager(self.master, self.target_system, self.target_component)

        self.add_command('rotor', self.cmd_auto, "rotor control", ['forward','backward'])

    def wait_motor(self, seconds):
        self.motor_event_complete = motor_event(seconds)

    def stop_motor(self):
        override_stop = [1500] * 16
        chan8 = self.override_stop[:8]
        self.master.mav.rc_channels_override_send(self.target_system,self.target_component,*chan8)

    def yaxis_motor(self, speed, seconds):
        '''control the bottom 4 motors fwd/rev'''
        offset = speed - 1500
        cw_speed = 1500 - offset
        args = ["1",str(speed)]
        self.rc_manager(args)
        args = ["2",str(speed)]
        self.rc_manager(args)
        args = ["3",str(speed)]
        self.rc_manager(args)
        args = ["4",str(speed)]
        self.rc_manager(args)

        self.wait_motor(seconds)

    def xaxis_motor(self, speed, seconds):
        '''control the bottom 4 motors left/right'''
        offset = speed - 1500
        cw_speed = 1500 - offset
        args = ["1",str(speed)]
        self.rc_manager(args)
        args = ["2",str(cw_speed)]
        self.rc_manager(args)
        args = ["3",str(cw_speed)]
        self.rc_manager(args)
        args = ["4",str(speed)]
        self.rc_manager(args)

        self.wait_motor(seconds)

    def zaxis_motor(self, speed, seconds):
        '''control the bottom 4 motors left/right'''
        offset = speed - 1500
        cw_speed = 1500 - offset
        args = ["5",str(speed)]
        self.rc_manager(args)
        args = ["6",str(cw_speed)]

        self.wait_motor(seconds)

    def roll_motor(self, speed, seconds):
        '''control the bottom 4 motors left/right'''

        args = ["5",str(speed)]
        self.rc_manager(args)
        args = ["6",str(speed)]

        self.wait_motor(seconds)

    def yaw_motor(self, speed, seconds):
        '''control the bottom 4 motors left/right'''

        offset = speed - 1500
        cw_speed = 1500 - offset
        args = ["1",str(speed)]
        self.rc_manager(args)
        args = ["2",str(cw_speed)]
        self.rc_manager(args)
        args = ["3",str(speed)]
        self.rc_manager(args)
        args = ["4",str(cw_speed)]
        self.rc_manager(args)

        self.wait_motor(seconds)

class motor_event(object):
    '''a class for fixed frequency events'''
    def __init__(self, seconds):
        self.seconds = seconds
        self.curr_time = time.time()
        self.final_time = curr_time + seconds

    def force(self):
        '''force immediate triggering'''
        self.curr_time = 0

    def trigger(self):
        '''return True if we should trigger now'''
        tnow = time.time()

        if tnow < self.curr_time:
            print("Warning, time moved backwards. Restarting timer.")
            tnow = self.curr_time

        if tnow >= self.final_time:
            self.last_time = tnow
            return True
        return False
