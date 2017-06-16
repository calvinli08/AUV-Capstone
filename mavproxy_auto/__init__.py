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
from MAVProxy.modules.mavproxy_auto import mp_waypoint
from MAVProxy.modules.mavproxy_auto import mp_rc
from MAVProxy.modules.mavproxy_auto import mp_fence

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

        self.battery_level = -1
        self.voltage_level = -1
        self.current_battery = -1

        self.pressure_sensor = [0] * 3
        self.depth_sensor = [0] * 3

        self.motor_event_complete = None
        self.motor_event_enabled = False
        self.wp_manager = mp_waypoint.WPManager(self.master, self.target_system, self.target_component)
        self.rc_manager = mp_rc.RCManager(self.master, self.target_system, self.target_component)
        self.fence_manager = mp_fence.FenceManager(self.master, self.target_system,self.target_component,self.console)
        self.add_command('auto', self.cmd_auto, "Autonomous sampling traversal", ['surface','underwater','setfence', 'dense'])
        self.add_command('unittest', self.cmd_unittest, "unit tests", ['<1|2|3|4|5|6|7>'])

        '''Test variables'''
        self.enable_temp_poll = False
        self.last_poll = time.time()
        self.poll_interval = 7

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

    '''unit test delete later '''
    def test1(self):
        '''xmotor test'''
        '''move foward for 3 seconds'''
        self.xaxis_motor(1510,1)
        '''move backward for 3 seconds'''
        self.xaxis_motor(1490,1)

    '''unit test delete later '''
    def test2(self):
        '''ymotor test'''
        '''strafe left for 3 seconds'''
        self.yaxis_motor(1510,1)
        '''strafe right for 3 seconds'''
        self.yaxis_motor(1490,1)

    '''unit test delete later '''
    def test3(self):
        '''roll motor test'''
        '''roll cw  for 3 seconds'''
        self.roll_motor(1510,1)
        '''roll ccw for 3 seconds'''
        self.roll_motor(1490,1)

    '''unit test delete later '''
    def test4(self):
        '''yaw motor test'''
        '''turn left  for 3 seconds'''
        self.yaw_motor(1510,1)
        '''turn right for 3 seconds'''
        self.yaw_motor(1490,1)

    '''unit test delete later'''
    def test5(self):
        '''z motor test'''
        '''dive for 3 seconds'''
        self.zaxis_motor(1490,1)
        '''surface for 3 seconds'''
        self.zaxis_motor(1510,1)

    '''unit test delete later '''
    def test6(self):
        '''sensor polling'''
        self.enable_temp_poll = True

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

    def cmd_underwater(self):
        return "Not yet implemented"

    def cmd_geofence(self, args):
        return "Not yet implemented"

    def load_geofence_points(self, filename):
        self.fence_manager.cmd_fence(['load',filename])

    #Check that the AUV is facing the correct cardinal direction and correct it if necessary
    def orient_heading(self, intended_heading):
        while intended_heading > 5:
            if intended_heading > 0:
                self.cmd_move(['yaw' 1800 20])
                print "orienting!"
            else:
                self.cmd_move(['yaw' 1200 20])
                print "otherwise!"
        else:
            print "done orienting!"
            self.stop_motor()

    #traverse
    def traverse_and_sample(self, start_time, time = 1):
        self.cmd_move(['y' 1800 20])
        print "traversing!"
        self.stop_motor()
        return self.sample()

    '''test with default values, delete later'''
    #dense traverse function that is called when pollution is above a threshold
    def dense_traverse(self, start, end, forward_distance_to_edge = 10, loop_number = 3, heading = 0, forward_travel_distance = 5, sideways_distance = 2, current = 0):
        '''
        dense traverse makes more loops within a smaller area. It accomplishes this by traveling 1/5 the width of sparse traverse for it's width portion, and minute steps for it's length.
        '''

        print "orangutan"
        if forward_distance_to_edge < forward_travel_distance:
            forward_travel_distance = forward_distance_to_edge - 2
        elif loops < sideways_distance:
            sideways_distance = sideways_distance/2

        print "pie"
        start_time = int(time.time())
        left_direction = heading - 90
        right_direction = heading + 90

        self.orient_heading(right_direction)
        print "matilda"
        previous_direction = right_direction
        self.traverse_and_sample(sideways_distance)
        print "mockingbird"

        for j in xrange(distance):
            self.orient_heading(heading)
            self.traverse_and_sample()
            print "blueberry jam"
            previous_direction += 180
            self.orient_heading(previous_direction)
            self.traverse_and_sample(sideways_distance)

        self.orient_heading(heading)
        self.traverse_and_sample()
        previous_direction += 180
        self.orient_heading(previous_direction)
        self.traverse_and_sample(sideways_distance/2)

        return forward_travel_distance

    def wait_motor(self, seconds):
        self.motor_event_complete = motor_event(seconds).trigger()
        self.stop_motor()

    def stop_motor(self):
        override_stop = [1500] * 16
        chan8 = override_stop[:8]
        self.master.mav.rc_channels_override_send(self.target_system,self.target_component,*chan8)

    '''
    args = [direction, pwm, seconds]
    roll - 1
    pitch - 2
    throttle - 3
    yaw - 4
    forward - 6
    lateral - 7
    '''
    def cmd_move(self, args):
        if len(args) != 3:
            return "Usage: move <x|y|z|roll|yaw> pwm seconds"
        elif args[0] == "x":
            self.rc_manager.set_override([1500 1500 1500 1500 1500 1500 args[1] 1500])
            self.wait_motor(args[2])
            return
        elif args[0] == "y":
            self.rc_manager.set_override([1500 1500 args[1] 1500 1500 args[1] 1500 1500])
            self.wait_motor(args[2])
            return
        elif args[0] == "z":
            self.rc_manager.set_override([1500 1500 1500 1500 1500 1500 1500 1500])
            self.wait_motor(args[2])
            return
        elif args[0] == "roll":
            self.rc_manager.set_override([args[1] 1500 1500 1500 1500 1500 1500 1500])
            self.wait_motor(args[2])
            return
        elif args[0] == "yaw":
            self.rc_manager.set_override([1500 1500 1500 args[1] 1500 1500 1500 1500])
            self.wait_motor(args[2])
            return
        else:
            return "Usage: move <x|y|z|roll|yaw> pwm seconds"

    '''
    def yaxis_motor(self, speed, seconds):
        #control the bottom 4 motors fwd/rev
        self.rc_manager.set_override([speed,speed,speed,speed, 1500, 1500, 0, 0])
        self.motor_event_complete = self.motor_event(seconds)

    def yaxis_motor(self, speed, seconds):
        #control the bottom 4 motors fwd/rev
        self.wait_motor(seconds)

    def xaxis_motor(self, speed, seconds):
        #control the bottom 4 motors left/right
        self.rc_manager.set_override([speed,cw_speed,cw_speed,speed, 1500, 1500, 0, 0])
        self.wait_motor(seconds)

    def zaxis_motor(self, speed, seconds):
        #control the bottom 4 motors left/right
        self.rc_manager.set_override([1500,1500,1500,1500,speed,cw_speed,0,0])
        self.wait_motor(seconds)

    def roll_motor(self, speed, seconds):
        #control the bottom 4 motors left/right
        self.rc_manager.set_override([1500,1500,1500,1500,speed,speed,0,0])
        self.wait_motor(seconds)

    def yaw_motor(self, speed, seconds):
        #control the bottom 4 motors left/right
        self.rc_manager.set_override([speed,cw_speed,speed,cw_speed, 1500, 1500, 0, 0])
        self.wait_motor(seconds)

    def roll_motor(self, speed, seconds):
        self.rc_manager.set_override([])
        self.wait_motor(seconds)

    '''

    def idle_task(self):
        '''handle missing waypoints'''
        if self.wp_manager.wp_period.trigger():
            # cope with packet loss fetching mission
            if self.master is not None and self.master.time_since('MISSION_ITEM') >= 2 and self.wp_manager.wploader.count() < getattr(self.wp_manager.wploader,'expected_count',0):
                wps = self.wp_manager.missing_wps_to_request();
                print("re-requesting WPs %s" % str(wps))
                self.wp_manager.send_wp_requests(wps)
        if self.rc_manager.override_period.trigger():
            if (self.rc_manager.override != [ 0 ] * 16 or
                self.rc_manager.override != self.rc_manager.last_override or
                self.rc_manager.override_counter > 0):
                self.rc_manager.last_override = self.rc_manager.override[:]
                self.rc_manager.send_rc_override()
                if self.rc_manager.override_counter > 0:
                    self.rc_manager.override_counter -= 1
        if self.motor_event_enabled:
            if(self.motor_event_complete.trigger()):
                self.stop_motor()

    def sensor_update(self, SCALED_PRESSURE2):
        self.motor_event_complete = None
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
               #print("m.seq=%u expected_count=%u" % (m.seq, self.wploader.expected_count))
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
             True
         False



def init(mpstate):
    '''initialise module'''
     AUVModule(mpstate)
