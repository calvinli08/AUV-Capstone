#!/usr/bin/env python

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import numpy

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_auto import mp_waypoint
from MAVProxy.modules.mavproxy_auto import mp_rc
from MAVProxy.modules.mavproxy_auto import mp_fence
from MAVProxy.modules.mavproxy_auto import SerialReader


class AUVModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(AUVModule, self).__init__(mpstate, "auto", "Telemetry Data for autonomous navigation", public = True)

        self.next_wp = [] #lat,lng
        self.intended_heading = 0
        self.pollution_array = None #initialize later
        self.loops = 0
        self.xy = {'x': 0, 'y': 0}  # x,y


        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.relative_alt = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.hdg = 0

        self.battery_level = -1
        self.voltage_level = -1
        self.current_battery = -1

        self.pressure_sensor = [0] * 3
        self.depth_sensor = [0] * 3

        self.last_waypoint = None

        self.motor_event_complete = None
        self.motor_event_enabled = False
        self.wp_manager = mp_waypoint.WPManager(self.master, self.target_system, self.target_component)
        self.rc_manager = mp_rc.RCManager(self.master, self.target_system, self.target_component)
        self.fence_manager = mp_fence.FenceManager(self.master, self.target_system,self.target_component,self.console)
        self.add_command('auto', self.cmd_auto, "Autonomous sampling traversal", ['surface','underwater','setfence', 'dense'])
        self.add_command('move', self.cmd_move, "Movement", ['<x|y|z|roll|yaw>', 'pwm', 'seconds'])
        self.add_command('unittest', self.cmd_unittest, "unit tests", ['<1|2|3|4|5|6|7>'])
        self.sensor_reader = SerialReader.SerialReader()

    def usage(self):
        '''show help on command line options'''
        return "Usage: auto <dense|setfence|surface|underwater>"

    def cmd_auto(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "surface":
            '''Generate surface waypoints'''
            self.cmd_surface()
        elif args[0] == "underwater":
            print self.cmd_underwater()
        elif args[0] == "setfence":
            print self.cmd_geofence(args[1:])
        elif args[0] == "test":
            print self.cmd_unittest(args[1:])
        elif args[0] == "dense":
            self.dense_traverse()
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

    def cmd_unittest(self,args):
        if len(args) == 0:
            print self.usage()
        elif args[0] == "1":
            self.test1()
        elif args[0] == "2":
            self.test2()
        elif args[0] == "3":
            self.test3()
        elif args[0] == "4":
            self.test4()
        elif args[0] == "5":
            self.test5()
        elif args[0] == "6":
            self.test6()
        elif args[0] == "7":
            self.test7()
        elif args[0] == "R":
             print(self.sensor_reader.read(0))
        return

    '''unit test delete later '''
    def test1(self):
        '''xmotor test'''
        '''move foward for 3 seconds'''
        self.cmd_move(['x', 1550, 1])
        self.cmd_move(['x', 1550, 1])
        '''move backward for 3 seconds'''

    '''unit test delete later '''
    def test2(self):
        '''ymotor test'''
        '''strafe left for 3 seconds'''
        self.cmd_move(['y', 1550, 1])
        '''strafe right for 3 seconds'''
        self.cmd_move(['y', 1450, 1])

    '''unit test delete later '''
    def test3(self):
        '''roll motor test'''
        '''roll cw  for 3 seconds'''
        self.cmd_move(["roll", 1550, 1])
        '''roll ccw for 3 seconds'''
        self.cmd_move(["roll", 1450, 1])

    '''unit test delete later '''
    def test4(self):
        '''yaw motor test'''
        '''turn left  for 3 seconds'''
        self.cmd_move(["yaw", 1550, 1])
        '''turn right for 3 seconds'''
        self.cmd_move(["yaw", 1450, 1])

    '''unit test delete later'''
    def test5(self):
        '''z motor test'''
        '''dive for 3 seconds'''
        # self.zaxis_motor(1450,1)
        self.cmd_move(['z',1450,1])
        self.cmd_move(['z',1550,1])
        '''surface for 3 seconds'''
        # self.zaxis_motor(1550,1)

    '''unit test delete later '''
    def test6(self):
        '''sensor polling'''
        self.cmd_move(['x',1550,3])
        self.cmd_move(["yaw",1550,2])
        self.cmd_move(['x',1550,5])
        self.cmd_move(["yaw",1550,5])

    '''unit test delete later '''
    def test7(self):
        '''waypoint testing'''
        self.enable_temp_poll = True
        f = open('testfile.txt', 'w')
        f.write('QGC WPL 110\n')
        f.write('0  1   0   16  0.149999999999999994    0   0   0   40.5195286 -74.46088470000001 0 1\n')
        f.write('1  0   0   16  0.149999999999999994    0   0   0   40.5195286 -74.46088470000001 0 1\n')
        f.write('2  0   0   16  0.149999999999999994    0   0   0   40.5195286 -74.46088470000001 0 1\n')
        f.close()

        args = ["load" , "testfile.txt"]
        self.wp_manager.cmd_wp(args)

    def cmd_underwater(self, args):
        if len(args) != 1:
            return "Usage: auto underwater start"
        elif args[0] == "start":
            self.run()
        else:
            return "Usage: auto underwater start"

    def run(self):
        mav.set_mode_manual()
        rc.set_mode_manual()
        mav.motors_armed_wait()
        #set the apm mav_type
        mav.mode_mapping()

        if self.predive_check() is not True:
            return "Insufficient Battery"

        self.distance_to_waypoint = mp_util.gps_distance(self.lat, self.lon,
                                                         self.next_wp.MAVLink_mission_item_message.x, self.next_wp.MAVLink_mission_item_message.y)

        self.intended_heading = mp_util.gps_bearing(self.lat, self.lon,
                                                    self.next_wp.MAVLink_mission_item_message.x, self.next_wp.MAVLink_mission_item_message.y)

        self.orient_heading(self.intended_heading)

        array_edges = self.calculate_geofence_edge_lengths()
        self.pollution_array = numpy.zeros([array_edges[0], array_edges[1]], float, 'C')  # each square meter is a point

        self.dive()

        # x = lat, y = lng
        self.underwater_traverse([self.lng, self.lat],
                                 [self.next_wp.MAVLink_mission_item_message.y,                       self.next_wp.MAVLink_mission_item_message.x],
                                 self.distance_to_waypoint, heading)

        self.surface()
        return

    def cmd_geofence(self, args):
        return "Not yet implemented"

    def calculate_geofence_edge_lengths(self):
        '''calculate length of geofence rectangle sides'''
        f = open('fence.txt', "r")
        points = []
        p = []
        for line in f:
            p = f.readline().split()
            points.append([p[0], p[1]])
        distance_between_points = []
        for x in range(len(points)-1):
            distance_between_points.append(mp_util.gps_distance(points[x][0], points[x][1], points[x+1][0], points[x+1][1]))
        return ( min(distance_between_points), max(distance_between_points) ) #width, length

    def load_geofence_points(self, filename):
        self.fence_manager.cmd_fence(['load',filename])

    def orient_heading(self, intended_heading):
        while intended_heading > 5:
            if intended_heading > 0:
                self.cmd_move(['yaw', 1800, 1])
                print("orienting!")
            else:
                self.cmd_move(['yaw', 1200, 1])
                print("otherwise!")
        else:
            print("done orienting!")
            return

    def surface(self, time):
        self.cmd_move(['z', 1600, time])
        return

    def dive(self, time = 1):
        self.cmd_move(['z', 1400, time])
        return

    # traverse
    def traverse(self, time = 1):
        self.cmd_move(['y', 1800, time])
        print "traversing!"
        return

    def sample(self, channel = 1):
        pollution_array[self.xy['x']][self.xy['y']] = self.sensor_reader.read(channel)
        return

    # underwater sparse traverse function
    def underwater_traverse(self, start, end, distance, heading, current = 1):
        start_time = int(time.time())
        end_time = int(time.time()) + distance + 10 #seconds
        '''Measure the run times and order of how this code segment runs'''
        while end_time - int(time.time()) >= 0:
            if self.traverse(time.time()) > threshold:
                elapsed_time = (int(time.time()) - start_time) + start_time
                remaining_distance = end_time - self.dense_traverse(elapsed_time, elapsed_time+5, end_time - int(time.time()), self.loops, heading, 5, 2, 1) - elapsed_time
                end_time = int(time.time()) + remaining_distance + 10
        else:
            self.stop_motor()
            if distance == 1:
                self.loops += 1
            return

    '''test with default values, delete later'''
    #dense traverse function that is called when pollution is above a threshold
    def dense_traverse(self, start, end, channel = 1, forward_distance_to_edge = 10, loop_number = 3, heading = 0, forward_travel_distance = 5, sideways_distance = 2, current = 0):

        print "ONE"
        if forward_distance_to_edge < forward_travel_distance:
            forward_travel_distance = forward_distance_to_edge - 2
        elif loops < sideways_distance:
            sideways_distance = sideways_distance/2

        print "TWO"
        start_time = int(time.time())
        left_direction = heading - 90
        right_direction = heading + 90

        self.orient_heading(right_direction)
        print "THREE"
        previous_direction = right_direction
        self.traverse(channel, sideways_distance)
        print "FOUR"

        for j in xrange(distance):
            self.orient_heading(heading)
            self.traverse(channel)
            print "FIVE"
            previous_direction += 180
            self.orient_heading(previous_direction)
            self.traverse(channel, sideways_distance)

        self.orient_heading(heading)
        self.traverse()
        previous_direction += 180
        self.orient_heading(previous_direction)
        self.traverse(channel, sideways_distance/2)

        return forward_travel_distance

    def wait_motor(self, seconds):
        self.motor_event_complete = motor_event(seconds)
        while self.motor_event_complete.trigger() is False:
            start_time = int(time.time())
            print("Waiting")
            time.sleep(0.5 - (int(time.time()) - start_time) % 0.5)
        self.stop_motor()

    def track_xy(self, pwm, direction):
        if direction in ['x', 'y']:
            sign = numpy.sign(pwm - 1500)
            while self.motor_event_enabled and self.loop%2 == 0:
                start_time = int(time.time())
                self.xy[direction] += sign
                self.sample(channel)
                time.sleep(60 - (int(time.time()) - start_time) % 60)
            else:
                self.xy['x'] = len(self.pollution_array)
            while self.motor_event_enabled and self.loop%2 == 1:
                start_time = int(time.time())
                self.xy[direction] -= sign
                self.sample(channel)
                time.sleep(60 - (int(time.time()) - start_time) % 60)


    # args = [direction, pwm, seconds]
    # roll - 1
    # pitch - 2
    # throttle - 3
    # yaw - 4
    # forward - 6
    # lateral - 7
    # source: https://github.com/bluerobotics/ardusub/blob/master/libraries/AP_RCMapper/AP_RCMapper.cpp
    def cmd_move(self, args):
        if len(args) != 3:
            return "Usage: move <x|y|z|roll|yaw> pwm seconds"
        elif args[0] == "x":
            self.rc_manager.override[6] = int(args[1])
            self.rc_manager.send_rc_override()
            self.wait_motor(int(args[2]))
            self.track_xy(int(args[1]), 'x')
            return
        elif args[0] == "y":
            # This is how the joystick module does it
            self.rc_manager.override[5] = int(args[1])
            self.rc_manager.send_rc_override()
            self.wait_motor(int(args[2]))
            self.track_xy(int(args[1]), 'y')
            return
        elif args[0] == "z":
            self.rc_manager.override[2] = int(args[1])
            self.rc_manager.send_rc_override()
            self.wait_motor(int(args[2]))
            return
        elif args[0] == "roll":
            self.rc_manager.override[0] = int(args[1])
            self.rc_manager.send_rc_override()
            self.wait_motor(int(args[2]))
            return
        elif args[0] == "yaw":
            self.rc_manager.override[3] = int(args[1])
            self.rc_manager.send_rc_override()
            self.wait_motor(int(args[2]))
            return
        elif args[0] == "pitch":
            self.rc_manager.override[1] = int(args[1])
            self.rc_manager.send_rc_override()
            self.wait_motor(int(args[2]))
            return
        else:
            return "Usage: move <x|y|z|roll|yaw> pwm seconds"

    def stop_motor(self):
        args = ["all", "1500"]
        self.rc_manager.cmd_rc(args)

    def idle_task(self):
        '''handle missing waypoints'''
        if self.wp_manager.wp_period.trigger():
            # cope with packet loss fetching mission
            if self.master is not None and self.master.time_since('MISSION_ITEM') >= 2 and self.wp_manager.wploader.count() < getattr(self.wp_manager.wploader,'expected_count',0):
                wps = self.wp_manager.missing_wps_to_request()
                print("re-requesting WPs %s" % str(wps))
                self.wp_manager.send_wp_requests(wps)
        if self.rc_manager.override_period.trigger():
            if (self.rc_manager.override != [ 1500 ] * 16 or
                self.rc_manager.override != self.rc_manager.last_override or
                self.rc_manager.override_counter > 0):
                self.rc_manager.last_override = self.rc_manager.override[:]
                self.rc_manager.send_rc_override()
                if self.rc_manager.override_counter > 0:
                    self.rc_manager.override_counter -= 1
        if self.motor_event_enabled:
            if self.motor_event_complete.trigger():
                self.stop_motor()
        self.sample(channel)

    def sensor_update(self, SCALED_PRESSURE2):
        if self.enable_temp_poll:
            now = time.time()
            if(now - self.last_poll > self.poll_interval):
                self.last_poll = now
                self.say("Depth sensor values: Press_abs: %f Press_diff: %f temperature: %d \n" % (self.depth_sensor[0],self.depth_sensor[1],self.depth_sensor[2]))
                self.say("Pressure sensor values: Press_abs: %f Press_diff: %f temperature: %d \n" % (self.pressure_sensor[0],self.pressure_sensor[1],self.pressure_sensor[2]))


    def psensor_update(self, SCALED_PRESSURE2):
        '''update pressure sensor readings'''
        self.pressure_sensor[0] = SCALED_PRESSURE2.press_abs
        self.pressure_sensor[1] = SCALED_PRESSURE2.press_diff
        self.pressure_sensor[2] = SCALED_PRESSURE2.temperature

    def dsensor_update(self, SCALED_PRESSURE):
        '''update depth sensor readings'''
        self.depth_sensor[0] = SCALED_PRESSURE.press_abs
        self.depth_sensor[1] = SCALED_PRESSURE.press_diff
        self.depth_sensor[2] = SCALED_PRESSURE.temperature

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

    def battery_update(self, SYS_STATUS):
        '''update battery level'''
        # main flight battery
        self.battery_level = SYS_STATUS.battery_remaining
        self.voltage_level = SYS_STATUS.voltage_battery
        self.current_battery = SYS_STATUS.current_battery


    def mavlink_packet(self, m):
        '''handle mavlink packets'''

        '''waypoint packets'''
        mtype = m.get_type()
        if mtype == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.gps_update(m)

        elif mtype == 'SCALED_PRESSURE2':
             self.psensor_update(m)

        elif mtype == 'SCALED_PRESSURE':
             self.dsensor_update(m)

        elif mtype == "SYS_STATUS":
            self.battery_update(m)

        if mtype in ['WAYPOINT_COUNT','MISSION_COUNT']:
           if self.wp_op is None:
               self.console.error("No waypoint load started")
           else:
               self.wploader.clear()
               self.wploader.expected_count = m.count
               self.console.writeln("Requesting %u waypoints t=%s now=%s" % (m.count,
                                                                                time.asctime(time.localtime(m._timestamp)),
                                                                                time.asctime()))
               self.send_wp_requests()

        elif mtype in ['WAYPOINT', 'MISSION_ITEM'] and self.wp_op != None:
            if m.seq < self.wploader.count():
                #print("DUPLICATE %u" % m.seq)
                return
            if m.seq+1 > self.wploader.expected_count:
                self.console.writeln("Unexpected waypoint number %u - expected %u" % (m.seq, self.wploader.count()))
            self.wp_received[m.seq] = m
            next_seq = self.wploader.count()
            while next_seq in self.wp_received:
                m = self.wp_received.pop(next_seq)
                self.wploader.add(m)
                next_seq += 1
            if self.wploader.count() != self.wploader.expected_count:
                # print("m.seq=%u expected_count=%u" % (m.seq, self.wploader.expected_count))
                self.send_wp_requests()
                return
            if self.wp_op == 'list':
                for i in range(self.wploader.count()):
                    w = self.wploader.wp(i)
                    print("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f p4=%.1f cur=%u auto=%u" % (
                       w.command, w.frame, w.x, w.y, w.z,
                       w.param1, w.param2, w.param3, w.param4,
                       w.current, w.autocontinue))
                if self.logdir != None:
                    waytxt = os.path.join(self.logdir, 'way.txt')
                    self.save_waypoints(waytxt)
                    print("Saved waypoints to %s" % waytxt)
            elif self.wp_op == "save":
                self.save_waypoints(self.wp_save_filename)
            self.wp_op = None
            self.wp_requested = {}
            self.wp_received = {}

        elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
           self.process_waypoint_request(m, self.master)

        elif mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
           if m.seq != self.last_waypoint:
               self.last_waypoint = m.seq
               if self.settings.wpupdates:
                   self.say("waypoint %u" % m.seq,priority='message')

        elif mtype == "MISSION_ITEM_REACHED":
            wp = self.wploader.wp(m.seq)
            if wp is None:
                # should we spit out a warning?!
                # self.say("No waypoints")
                pass
            else:
                if wp.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                    alt_offset = self.get_mav_param('ALT_OFFSET', 0)
                    if alt_offset > 0.005:
                        self.say("ALT OFFSET IS NOT ZERO passing DO_LAND_START")
                self.next_wp = wp

        elif m.get_type() == "FENCE_STATUS":
            self.fence_manager.last_fence_breach = m.breach_time
            self.fence_manager.last_fence_status = m.breach_status
        elif m.get_type() in ['SYS_STATUS']:
            bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

            present = ((m.onboard_control_sensors_present & bits) == bits)
            if self.fence_manager.present == False and present == True:
                self.say("fence present")
            elif self.fence_manager.present == True and present == False:
                self.say("fence removed")
            self.present = present

            enabled = ((m.onboard_control_sensors_enabled & bits) == bits)
            if self.fence_manager.enabled == False and enabled == True:
                self.say("fence enabled")
            elif self.fence_manager.enabled == True and enabled == False:
                self.say("fence disabled")
            self.fence_manager.enabled = enabled

            healthy = ((m.onboard_control_sensors_health & bits) == bits)
            if self.fence_manager.healthy == False and healthy == True:
                self.say("fence OK")
            elif self.fence_manager.healthy == True and healthy == False:
                self.say("fence breach")
            self.fence_manager.healthy = healthy

            #console output for fence:
            if self.fence_manager.enabled == False:
                self.fence_manager.console.set_status('Fence', 'FEN', row=0, fg='grey')
            elif self.fence_manager.enabled == True and self.fence_manager.healthy == True:
                self.console.set_status('Fence', 'FEN', row=0, fg='green')
            elif self.fence_manager.enabled == True and self.fence_manager.healthy == False:
                self.console.set_status('Fence', 'FEN', row=0, fg='red')


class motor_event(object):
    '''a class for fixed frequency events'''
    def __init__(self, seconds):
        self.seconds = seconds
        self.curr_time = time.time()
        self.final_time = self.curr_time + seconds

    def force(self):
        '''force immediate triggering'''
        self.curr_time = 0

    def trigger(self):
        ''' True if we should trigger now'''
        tnow = time.time()

        if tnow < self.curr_time:
            print("Warning, time moved backwards. Restarting timer.")
            tnow = self.curr_time

        if tnow >= self.final_time:
            self.last_time = tnow
            return True
        return False


def init(mpstate):
    '''initialise module'''
    return AUVModule(mpstate)
