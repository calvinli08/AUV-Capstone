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
from MAVProxy.modules import mavproxy_rc as rc
from MAVProxy.modules import mavproxy_devop as dp
from MAVProxy.modules.mavproxy_auto import mp_waypoint

class AUVModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(AUVModule, self).__init__(mpstate, "auto", "Telemetry Data for autonomous navigation", public = True)
        radctrl = rc()
        self.distance_to_waypoint =
        self.intended_heading =

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
        for waypoint in wherever_waypoints are:
            mav.set_mode_manual()
            radctrl.set_mode_manual()
            mav.motors_armed_wait()
            #set the apm mav_type
            mav.mode_mapping()

            if self.predive_check() is not True:
                return "Insufficient Battery"

            self.orient_heading(self.intended_heading)

            self.dive([self.lng, self.lat], waypoint, distance, depth = 1, self.intended_heading current = 1)

            self.underwater_traverse([self.lng, self.lat], waypoint, distance, heading)

            self.surface()

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
        self.distance_to_waypoint = mp_util.gps_distance(self.lat, self.lon, ,)
        self.intended_heading = mp_util.gps_bearing(self.lat, self.lon, ,)

    #Performs pre-dive information gathering and checks
    def predive_check(self):
        #calibrate the pressure
        mav.calibrate_pressure()
        if self.battery >= 60.0:
            return True
        elif 35.0 <= self.battery < 60.0 and (self.battery*time_multiplier - (distance / self.velocity)) > 2:
            return True
        else:
            return False

    #Dives to a set depth
    def dive(self, dive_point, destination, distance, depth = 1, heading, current = 1):
        for i in xrange(0, depth, 0.5):
            radctrl.set_override([0 0 0 0 1200 1800 0 0])
        radctrl.set_override([0 0 0 0 0 0 0 0])
        self.surfaced = False

    #Surface
    def surface(self):
        for i in xrange(0, depth, 0.5)
            radctrl.set_override([0 0 0 0 1800 1200 0 0])
        radctrl.set_override([0 0 0 0 0 0 0 0])
        mav.set_mode_apm(9)#set mode to surfaced
        self.surfaced = True
        return

    #Check that the AUV is facing the correct cardinal direction and correct it if necessary
    def orient_heading(self, intended_heading):
        while intended_heading > 5:
            if intended_heading > 0:
                radctrl.set_override([1800 0 0 1800 0 0 0 0])#change yaw ccw
            else:
                radctrl.set_override([0 1800 1800 0 0 0 0 0])#change yaw cw
        else:
            radctrl.set_override([0 0 0 0 0 0 0 0])

    #Sample pollution with the sensors
    def sample(self):
        sensor_data = dp.devop_read()
        self.pollution.append(sensor_data)
        return sensor_data
    #traverse
    def traverse_and_sample(self, time = 1):
        while mav.motors_armed():
            radctrl.set_override([1800 1800 1200 1200 0 0 0 0])#move forward at 1 m/s
            radctrl.set_override([0 0 0 0 0 0 0 0])#spin thrusters in opposite direction of current
        return self.sample()

    #underwater sparse traverse function
    def underwater_traverse(self, start, end, distance, heading, current = 1):
        end_time = time.clock() + distance + 10 #seconds
        '''Measure the run times and order of how this code segment runs'''
        while end_time - time.clock() >= 0:
                if self.traverse_and_sample() > threshold:
                    remaining_distance = end_time - self.dense_traverse(,, '''Calculate distance based on ''', heading, 1)
                    end_time = time.clock() + remaining_distance
                else:
                #Once time is elapsed, stop
                self.velocity = 0
                return

    #dense traverse function that is called when pollution is above a threshold
    def dense_traverse(self, start, end, distance, heading, current = 1):
        '''
        dense traverse makes more loops within a smaller area. It accomplishes this by traveling 1/5 the width of sparse traverse for it's width portion, and minute steps for it's length.
        '''

        start_time = time.clock()
        left_direction = heading - 90
        right_direction = heading + 90

        self.orient_heading(left_direction)
        previous_direction = left_direction
        self.traverse_and_sample(distance/2)

        self.orient_heading(heading)
        self.traverse_and_sample()

        for j in xrange(distance):
            self.orient_heading(heading)
            self.traverse_and_sample()

            previous_direction = self.direction_flip(previous_direction)
            self.orient_heading(previous_direction)
            self.traverse_and_sample(distance)

        self.orient_heading(heading)
        self.traverse_and_sample()
        self.orient_heading(self.direction_flip(previous_direction))
        self.traverse_and_sample(distance/2)

    def direction_flip(self, direction):
        return direction += 180


    def mavlink_packet(self, m):
        '''handle mavlink packets'''
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




def init(mpstate):
    '''initialise module'''
    return AUVModule(mpstate)
