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
from MAVProxy.modules.mavproxy_auv import mp_waypoint

class AUVModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(AUVModule, self).__init__(mpstate, "auto", "Telemetry Data for autonomous navigation", public = True)

        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.relative_alt = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.hdg = 0

        self.pressure_sensor = [0] * 3

        self.wp_manager = mp_waypoint.WPManager(self.master, self.target_system, self.target_component)

        self.add_command('auto', self.cmd_auto, "Autonomous sampling traversal", ['surface','underwater'])


    def usage(self):
        '''show help on command line options'''
        return "Usage: auto <surface|underwater>"

    def cmd_auto(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "surface":
            '''Generate surface waypoints'''
            self.cmd_surface()
        elif args[0] == "underwater":
            print self.cmd_underwater()
        else:
            print self.usage()

    def cmd_surface(self):
        '''Generate waypoints'''
        '''Toy test'''
        f = open('testfile.txt', 'w')
        f.write('QGC WPL 110\n')
        f.write('0  1   0   16  0.149999999999999994    0   0   0   8.54800000000000004 47.3759999999999977 550 1\n')
        f.write('1  0   0   16  0.149999999999999994    0   0   0   8.54800000000000004 47.3759999999999977 550 1\n')
        f.write('2  0   0   16  0.149999999999999994    0   0   0   8.54800000000000004 47.3759999999999977 550 1\n')
        f.close()

        args = ["load" , "testfile.txt"]
        self.wp_manager.cmd_wp(args)

    def cmd_underwater(self):
        return "Not yet implemented"

    def idle_task(self):
        '''handle missing waypoints'''
        if self.wp_manager.wp_period.trigger():
            # cope with packet loss fetching mission
            if self.master is not None and self.master.time_since('MISSION_ITEM') >= 2 and self.wp_manager.wploader.count() < getattr(self.wp_manager.wploader,'expected_count',0):
                wps = self.wp_manager.missing_wps_to_request();
                print("re-requesting WPs %s" % str(wps))
                self.wp_manager.send_wp_requests(wps)

    def sensor_update(self, SCALED_PRESSURE2):
        '''update pressure sensor readings'''
        self.pressure_sensor[0] = SCALED_PRESSURE2.press_abs
        self.pressure_sensor[1] = SCALED_PRESSURE2.press_diff
        self.pressure_sensor[2] = SCALED_PRESSURE2.temperature

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

    def mavlink_packet(self, m):
        '''handle mavlink packets'''

        '''waypoint packets'''
        mtype = m.get_type()
        if mtype == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.gps_update(m)

        elif mtype == 'SCALED_PRESSURE2':
             self.sensor_update(m)

        elif mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
            self.wp_manager.wploader.expected_count = m.count
            if self.wp_manager.wp_op is None:
                self.wp_manager.console.error("No waypoint load started")
            else:
                self.wp_manager.wploader.clear()
                self.wp_manager.console.writeln("Requesting %u waypoints t=%s now=%s" % (m.count,
                                                                                 time.asctime(time.localtime(m._timestamp)),
                                                                                 time.asctime()))
                self.wp_manager.send_wp_requests()

        elif mtype in ['WAYPOINT', 'MISSION_ITEM'] and self.wp_manager.wp_op != None:
            if m.seq < self.wp_manager.wploader.count():
                #print("DUPLICATE %u" % m.seq)
                return
            if m.seq+1 > self.wp_manager.wploader.expected_count:
                self.wp_manager.console.writeln("Unexpected waypoint number %u - expected %u" % (m.seq, self.wp_manager.wploader.count()))
            self.wp_manager.wp_received[m.seq] = m
            next_seq = self.wp_manager.wploader.count()
            while next_seq in self.wp_manager.wp_received:
                m = self.wp_manager.wp_received.pop(next_seq)
                self.wp_manager.wploader.add(m)
                next_seq += 1
            if self.wp_manager.wploader.count() != self.wp_manager.wploader.expected_count:
                #print("m.seq=%u expected_count=%u" % (m.seq, self.wp_manager.wploader.expected_count))
                self.wp_manager.send_wp_requests()
                return
            if self.wp_manager.wp_op == 'list':
                for i in range(self.wp_manager.wploader.count()):
                    w = self.wp_manager.wploader.wp(i)
                    print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                        w.command, w.frame, w.x, w.y, w.z,
                        w.param1, w.param2, w.param3, w.param4,
                        w.current, w.autocontinue))
                if self.wp_manager.logdir != None:
                    waytxt = os.path.join(self.wp_manager.logdir, 'way.txt')
                    self.wp_manager.save_waypoints(waytxt)
                    print("Saved waypoints to %s" % waytxt)
            elif self.wp_manager.wp_op == "save":
                self.wp_manager.save_waypoints(self.wp_manager.wp_save_filename)
            self.wp_manager.wp_op = None
            self.wp_manager.wp_requested = {}
            self.wp_manager.wp_received = {}

        elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
            self.wp_manager.process_waypoint_request(m, self.wp_manager.master)

        elif mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
            if m.seq != self.wp_manager.last_waypoint:
                self.wp_manager.last_waypoint = m.seq
                if self.wp_manager.settings.wpupdates:
                    self.wp_manager.say("waypoint %u" % m.seq,priority='message')

        elif mtype == "MISSION_ITEM_REACHED":
            wp = self.wp_manager.wploader.wp(m.seq)
            if wp is None:
                # should we spit out a warning?!
                # self.wp_manager.say("No waypoints")
                pass
            else:
                if wp.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                    alt_offset = self.wp_manager.get_mav_param('ALT_OFFSET', 0)
                    if alt_offset > 0.005:
                        self.wp_manager.say("ALT OFFSET IS NOT ZERO passing DO_LAND_START")

        '''Fence packets'''

        if m.get_type() == "FENCE_STATUS":
            self.last_fence_breach = m.breach_time
            self.last_fence_status = m.breach_status
        elif m.get_type() in ['SYS_STATUS']:
            bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

            present = ((m.onboard_control_sensors_present & bits) == bits)
            if self.present == False and present == True:
                self.say("fence present")
            elif self.present == True and present == False:
                self.say("fence removed")
            self.present = present

            enabled = ((m.onboard_control_sensors_enabled & bits) == bits)
            if self.enabled == False and enabled == True:
                self.say("fence enabled")
            elif self.enabled == True and enabled == False:
                self.say("fence disabled")
            self.enabled = enabled

            healthy = ((m.onboard_control_sensors_health & bits) == bits)
            if self.healthy == False and healthy == True:
                self.say("fence OK")
            elif self.healthy == True and healthy == False:
                self.say("fence breach")
            self.healthy = healthy

            #console output for fence:
            if self.enabled == False:
                self.console.set_status('Fence', 'FEN', row=0, fg='grey')
            elif self.enabled == True and self.healthy == True:
                self.console.set_status('Fence', 'FEN', row=0, fg='green')
            elif self.enabled == True and self.healthy == False:
                self.console.set_status('Fence', 'FEN', row=0, fg='red')




def init(mpstate):
    '''initialise module'''
    return AUVModule(mpstate)
