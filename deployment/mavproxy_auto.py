#!/usr/bin/env python

import os
import os.path
import sys
from pymavlink import mavutil, mavwp
import errno
import time
import numpy as np

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules import mavproxy_rc
from MAVProxy.modules import mavproxy_devop
from MAVProxy.modules import mavproxy_fence
from MAVProxy.modules import mavproxy_wp
from MAVProxy.modules import mavproxy_rotors

class AutoModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(AutoModule, self).__init__(mpstate, "auto", "Telemetry Data for autonomous navigation", public = True)
        rc = mavproxy_rc.RCModule()
        fn = mavproxy_fence.FenceModule()
        rt = mavproxy_rotors.RotorsModule()
        self.distance_to_waypoint = 0
        self.next_wp = [] #lat,lng
        self.intended_heading = 0
        self.pollution_array = None #initialize later
        self.loops = 0

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

        self.add_command('auto', self.cmd_auto, "Autonomous sampling traversal", ['surface','underwater', 'geofence'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: auto <surface|underwater>"

    def cmd_geofence(self):
        '''control behavior of fence'''
        if len(args) == 0:
            print "geofence <load|reload>"
        elif args[0] == "load":
            if len(args) == 1:
                self.load_geofence_points()
            elif len(args) == 2:
                self.load_geofence_points(args[1])
                return
            else:
                print "usage: geofence load <filename>"
                return
        elif args[0] == "reload":
            if len(args) == 1:
                self.load_geofence_points()
            elif len(args) == 2:
                self.load_geofence_points(args[1])
                return
            else:
                print "usage: geofence reload <filename>"
                return
        else:
            print "geofence <load|reload>"

    def load_geofence_points(self, filename = '/home/pi/geofence.txt'):
        '''load fence points from file'''
        '''fence points are lat,lng coordinates'''
        fn.cmd_fence(['load', filename])

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
        width = min(distance_between_points)
        length = max(distance_between_points)

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

        mav.set_mode_manual()
        rc.set_mode_manual()
        mav.motors_armed_wait()
        #set the apm mav_type
        mav.mode_mapping()

        if self.predive_check() is not True:
            return "Insufficient Battery"

        self.distance_to_waypoint = mp_util.gps_distance(self.lat, self.lon, self.next_wp.MAVLink_mission_item_message.x, self.next_wp.MAVLink_mission_item_message.y)

        self.intended_heading = mp_util.gps_bearing(self.lat, self.lon, self.next_wp.MAVLink_mission_item_message.x, self.next_wp.MAVLink_mission_item_message.y)

        self.orient_heading(self.intended_heading)

        self.pollution_array = None

        #x = lat, y = lng
        self.dive([self.lng, self.lat], [self.next_wp.MAVLink_mission_item_message.y, self.next_wp.MAVLink_mission_item_message.x], self.distance_to_waypoint, self.intended_heading, 1, 1)

        self.underwater_traverse([self.lng, self.lat], [self.next_wp.MAVLink_mission_item_message.y, self.next_wp.MAVLink_mission_item_message.x], self.distance_to_waypoint, heading)

        self.surface()
        return

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

    def gps_update(self, GLOBAL_POSITION_INT, next_wp):
        '''update gps readings'''
        self.lat = GLOBAL_POSITION_INT.lat
        self.lon = GLOBAL_POSITION_INT.lon
        self.alt = GLOBAL_POSITION_INT.alt
        self.relative_alt = GLOBAL_POSITION_INT.relative_alt
        self.vx = GLOBAL_POSITION_INT.vx
        self.vy = GLOBAL_POSITION_INT.vy
        self.vz = GLOBAL_POSITION_INT.vz
        self.hdg = GLOBAL_POSITION_INT.hdg

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
    def dive(self, dive_point, destination, distance, heading, depth = 1, current = 1):
        #while time.time() - start_time < 0.7:
        rt.zaxis_motors(1800, 1.2)
            #rc.set_override([0 0 0 0 1200 1800 0 0])
        #rc.set_override([1500 1500 1500 1500 1500 1500 1500 1500])
        self.surfaced = False

    #Surface
    def surface(self):
       # for i in xrange(0, depth, 0.5)
        #    rc.set_override([0 0 0 0 1800 1200 0 0])
        #rc.set_override([1500 1500 1500 1500 1500 1500 1500 1500])
        rt.zaxis_motors(1200, 2)
        mav.set_mode_apm(9)#set mode to surfaced
        self.surfaced = True
        return

    #Check that the AUV is facing the correct cardinal direction and correct it if necessary
    def orient_heading(self, intended_heading):
        while intended_heading > 5:
            if intended_heading > 0:
                #rc.set_override([1800 0 0 1800 0 0 0 0])#change yaw ccw
                rt.yaw_motor(1300, 1)
            else:
                rt.yaw_motor(1800, 1)
                #rc.set_override([0 1800 1800 0 0 0 0 0])#change yaw cw
        else:
            #rc.set_override([1500 1500 1500 1500 1500 1500 1500 1500])
            rt.stop_motor()

    #Sample pollution with the sensors
    def sample(self):
        sensor_data = dp.devop_read()
        self.pollution.append(sensor_data)
        return sensor_data

    #traverse
    def traverse_and_sample(self, start_time, time = 1):
        #while mav.motors_armed() and time.time() - start_time < time:
            #rc.set_override([1800 1800 1200 1200 0 0 0 0])#move forward at 1 m/s
            #rc.set_override([1500 1500 1500 1500 1500 1500 1500 1500])#spin thrusters in opposite direction of current
        rt.yaxis_motor(1800, time)
        #else:
        rt.stop_motor()
            #rc.set_override([1500 1500 1500 1500 1500 1500 1500 1500])
        return self.sample()

    #underwater sparse traverse function
    def underwater_traverse(self, start, end, distance, heading, current = 1):
        start_time = int(time.time())
        end_time = int(time.time()) + distance + 10 #seconds
        '''Measure the run times and order of how this code segment runs'''
        while end_time - int(time.time()) >= 0:
            if self.traverse_and_sample(time.time()) > threshold:
                elapsed_time = (int(time.time()) - start_time) + start_time
                remaining_distance = end_time - self.dense_traverse(elapsed_time, elapsed_time+5, end_time - int(time.time()), self.loops, heading, 5, 2, 1) - elapsed_time
                end_time = int(time.time()) + remaining_distance + 10
        else:
            #Once time is elapsed, stop
            #rc.set_override([1500 1500 1500 1500 1500 1500 1500 1500])
            rt.stop_motor()
            if distance == 1:
                self.loops += 1
            return

    #dense traverse function that is called when pollution is above a threshold
    def dense_traverse(self, start, end, forward_distance_to_edge, loop_number, heading, forward_travel_distance = 5, sideways_distance = 2, current = 1):
        '''
        dense traverse makes more loops within a smaller area. It accomplishes this by traveling 1/5 the width of sparse traverse for it's width portion, and minute steps for it's length.
        '''

        if forward_distance_to_edge < forward_travel_distance:
            forward_travel_distance = forward_distance_to_edge - 2
        elif loops < sideways_distance:
            sideways_distance = sideways_distance/2


        start_time = int(time.time())
        left_direction = heading - 90
        right_direction = heading + 90

        self.orient_heading(right_direction)
        previous_direction = right_direction
        self.traverse_and_sample(sideways_distance)

        for j in xrange(distance):
            self.orient_heading(heading)
            self.traverse_and_sample()

            previous_direction += 180
            self.orient_heading(previous_direction)
            self.traverse_and_sample(sideways_distance)

        self.orient_heading(heading)
        self.traverse_and_sample()
        previous_direction += 180
        self.orient_heading(previous_direction)
        self.traverse_and_sample(sideways_distance/2)

        return forward_travel_distance

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
            self.go_home()
            #pass
          else:
            if wp.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                alt_offset = self.wp_manager.get_mav_param('ALT_OFFSET', 0)
                if alt_offset > 0.005:
                    self.wp_manager.say("ALT OFFSET IS NOT ZERO passing DO_LAND_START")
            else:
                self.next_wp = [wp.MAVLink_mission_item_message.x, wp.MAVLink_mission_item_message.y] #lat,lng
                self.cmd_underwater(wp)


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
    return AutoModule(mpstate)
