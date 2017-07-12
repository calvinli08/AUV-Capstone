#!/usr/bin/env python

import os
import os.path
import sys
import numpy
import errno
from pymavlink import mavutil
from time import strftime, time
from collections import deque
from math import sqrt, pow

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules import mp_waypoint
from MAVProxy.modules import mp_rc
from MAVProxy.modules import mp_fence
from MAVProxy.modules import SerialReader


class AUVModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(AUVModule, self).__init__(mpstate, "auto", "Autonomous navigation module")

        '''Navigational information'''
        self.next_wp = []  # lat,lng
        self.offset_from_intended_heading = 0
        self.pollution_array = numpy.zeros([2, 2])  # initialize later
        self.loops = 0
        self.xy = {'x': 0, 'y': 0}  # x,y

        '''Attitude'''
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.relative_alt = 0
        self.vx = 0
        self.vy = 1
        self.vz = 1
        self.hdg = 0

        '''Battery information'''
        self.battery_level = -1
        self.voltage_level = -1
        self.current_battery = -1
        self.last_batt = time()
        self.ujoules = 0
        self.write_to_battery = []

        '''Pressure and Depth Sensors'''
        self.temp_sensor = [0] * 3
        self.depth_sensor = [0] * 3

        self.last_waypoint = None

        '''Infinite sized queue for motor commands'''
        self.command_queue = deque()
        self.end_time = 0
        self.motor_run_time = 0

        self.last_sample = time()

        '''Instances of other modules'''
        self.wp_manager = mp_waypoint.WPManager(self.master, self.target_system, self.target_component)
        self.fence_manager = mp_fence.FenceManager(self.master, self.target_system, self.target_component, self.console)
        self.sensor_reader = SerialReader.SerialReader()

        ''' Commands for operating the module from the MAVProxy CLI'''
        self.add_command('auto', self.cmd_auto, "Autonomous sampling traversal", ['test', 'surface', 'underwater', 'setfence'])
        self.add_command('dense', self.cmd_dense, "dense traversal", ['start'])

        self.mav = mavutil.mavfile(None, None)  # only using set_mode_manual(), so fd and address can be None

    def usage(self):
        '''show help on command line options'''
        return "Usage: auto <test|setfence|surface|underwater>"

    def cmd_auto(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "surface":
            '''Generate surface waypoints'''
            self.cmd_surface()
        elif args[0] == "underwater":
            self.cmd_underwater(args[1:])
        elif args[0] == "setfence":
            print self.cmd_geofence(args[1:])
        elif args[0] == "test":
            print self.test1()
        else:
            print self.usage()

    def cmd_dense(self, args):
        if len(args) == 0:
            return "Usage: dense start forward_increment
        elif args[0] == 'start':
            self.dense_traverse(int(args[1]))
        else:
            return "Usage: dense forward_increment"

    def cmd_surface(self):
        '''Generate waypoints'''
        '''Toy test'''
        f = open('testfile.txt', 'w')
        f.write('QGC WPL 110\n')
        f.write('0  1   0   16  0.149999999999999994    0   0   0   8.54800000000000004 47.3759999999999977 550 1\n')
        f.write('1  0   0   16  0.149999999999999994    0   0   0   8.54800000000000004 47.3759999999999977 550 1\n')
        f.write('2  0   0   16  0.149999999999999994    0   0   0   8.54800000000000004 47.3759999999999977 550 1\n')
        f.close()

        args = ["load", "testfile.txt"]
        self.wp_manager.cmd_wp(args)

    def cmd_underwater(self, args):
        if args[0] == "start":
            self.run()
        else:
            return "Usage: auto underwater start"

    '''unit test delete later '''
    def test1(self):
        '''yaw motor test'''

        '''yaw clockwise for 3 seconds'''
        self.command_queue.append(["y", 1540, 3])

        '''yaw ccw for 3 seconds'''
        self.command_queue.append(["y", 1450, 3])

    '''Performs pre-dive information gathering and checks'''
    def predive_check(self):
        if self.next_wp == None:
            raise ValueError('self.next_wp is None')
        if self.battery >= 60.0:
            return
        elif 35.0 <= self.battery < 60.0 and (self.battery*time_multiplier - (distance / self.velocity)) > 2:
            return
        else:
            raise HardwareError([1, 'Insufficient Battery'])

    def run(self):
        try:
            self.predive_check()

            self.distance_to_waypoint = mp_util.gps_distance(self.lat, self.lon,
                                                             self.next_wp.MAVLink_mission_item_message.x, self.next_wp.MAVLink_mission_item_message.y)

            self.offset_from_intended_heading = mp_util.gps_bearing(self.lat, self.lon,
                                                        sel"yaw"f.next_wp.MAVLink_mission_item_message.x, self.next_wp.MAVLink_mission_item_message.y)

            self.orient_heading(self.offset_from_intended_heading)

            array_edges = self.calculate_geofence_edge_lengths()
            self.pollution_array = numpy.zeros([array_edges[0], array_edges[1]], float, 'C')  # each square meter is a point

            self.dive()

            # x = lat, y = lng
            self.underwater_traverse([self.lng, self.lat],
                                     [self.next_wp.MAVLink_mission_item_message.y,                        self.next_wp.MAVLink_mission_item_message.x],
                                     self.distance_to_waypoint, heading)

            self.surface()

        except HardwareError:
            return HardwareError.message

        except ValueError:
            sleep(60)

        numpy.savetxt('pollution_array.txt', self.pollution_array)
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
        return (min(distance_between_points), max(distance_between_points))  # width, length

    def load_geofence_points(self, filename):
        self.fence_manager.cmd_fence(['load', filename])

    def track_xy(self, pwm, direction):
        if direction in ['x', 'y']:
            sign = numpy.sign(pwm - 1500)
            while self.motor_event_enabled and self.loop % 2 == 0:
                start_time = int(time())
                self.xy[direction] += sign
                self.sample(channel)
                time.sleep(1 - (int(time()) - start_time) % 1)
            else:
                self.xy['x'] = len(self.pollution_array)
            if direction == 'y':
                sign *= -1
            while self.motor_event_enabled and self.loop % 2 == 1:
                start_time = int(time())
                self.xy[direction] -= sign
                self.sample(channel)
                time.sleep(1 - (int(time()) - start_time) % 1)

    # traverse
    # assuming: one second = one meter, 2 seconds delay
    def traverse(self, time=3):
        self.command_queue.append(["f", 1600, time])
        return

    def surface(self, time=5):
        self.command_queue.append(["z", 1650, time])
        return

    def dive(self, time=3):
        self.command_queue.append(["z", 1450, time])
        return

    def orient_heading(self, offset_from_intended_heading, pwm=1550):
        diff = abs(pwm - 1500)
        ccw_pwm = 1500 - diff
        cw_pwm = 1500 + diff
        if offset_from_intended_heading > 0:
            self.command_queue.append(["y", ccw_pwm, 2])
        else:
            self.command_queue.append(["y", cw_pwm, 2])


    # args = [direction, pwm, seconds]
    # roll - 3
    # z - 2
    # yaw - 4
    # forward - 5
    # lateral - 6
    def cmd_move(self, args):
        if len(args) != 3:
            return "Usage: move <f|l|z|roll|yaw> pwm"
        elif args[0] == "f":  # forward
            self.module('rc').override[4] = args[1]
            self.module('rc').send_rc_override()
            # self.track_xy(args[1], 'y')
            return
        elif args[0] == "l":  # lateral
            # This is how the joystick module does it
            self.module('rc').override[5] = args[1]
            self.module('rc').send_rc_override()
            # self.track_xy(args[1], 'y')
            return
        elif args[0] == "z":  # dive/surface
            self.module('rc').override[1] = args[1]
            self.module('rc').send_rc_override()
            return
        elif args[0] == "r":  # roll
            self.module('rc').override[2] = args[1]
            self.module('rc').send_rc_override()
            return
        elif args[0] == "y":  # yaw
            self.module('rc').override[3] = args[1]
            self.module('rc').send_rc_override()
            return
        else:
            return "Usage: move <f|l|z|r|y> pwm seconds"

    '''underwater sparse traverse function'''
    def underwater_traverse(self, start, end, distance, heading, current=1):
        start_time = int(time())
        end_time = int(time()) + distance + 1  # seconds
        '''Measure the run times and order of how this code segment runs'''
        self.traverse(distance+1)
        if self.sample():
            elapsed_time = (int(time()) - start_time) + start_time
            remaining_distance = end_time - self.dense_traverse() - elapsed_time
            end_time = int(time()) + remaining_distance + 1
        if distance == 1:
            self.loops += 1
        return

    ''' dense traverse function that is called when pollution is above a threshold'''
    def dense_traverse(self, forward_increment=3, pwm=1550, forward_distance_to_edge=10, loop_number=3, forward_travel_distance=5, sideways_distance=4, current=0):

        if forward_distance_to_edge < forward_travel_distance:
            forward_travel_distance = forward_distance_to_edge - 2
        elif self.loops < sideways_distance:
            sideways_distance = sideways_distance/2

        self.orient_heading(90, pwm)
        self.traverse(sideways_distance)

        turn_direction = -90
        for j in xrange(forward_travel_distance):
            self.orient_heading(turn_direction, pwm)
            self.traverse(forward_increment)
            self.orient_heading(turn_direction, pwm)
            self.traverse(sideways_distance)
            turn_direction *= -1

        self.orient_heading(turn_direction, pwm)
        self.traverse(3)
        self.orient_heading(turn_direction, pwm)
        self.traverse(sideways_distance/2)
        turn_direction *= -1
        self.orient_heading(turn_direction, pwm)

        return forward_travel_distance

    def batt_info(self):
        return float(self.current_battery) * float(self.voltage_level)  # micro-watts

    # test threshold is 0.7, real threshold value will be pulled from environmental data
    def sample(self):
        with open("/home/pi/sensor_battery.txt", "a+") as f:
            f.write("DO: %s, Cond: %s, Temp: %s, Lat: %s, Long: %s, uWatts: %s, Time: %s \n" % (self.sensor_reader.read("2").rstrip(), self.sensor_reader.read("3").rstrip(), self.temp_sensor[2], self.lat, self.lon, self.batt_info(), strftime("%H:%M:%S")))  # DO, Conductivity, Temperature, Lat, Lng, microWatts, time
        # pollution_array[self.xy['x']][self.xy['y']] = pollution_value
        return

    def psensor_update(self, SCALED_PRESSURE3):
        '''update pressure sensor readings'''
        self.temp_sensor[0] = SCALED_PRESSURE3.press_abs
        self.temp_sensor[1] = SCALED_PRESSURE3.press_diff
        self.temp_sensor[2] = SCALED_PRESSURE3.temperature

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
        # self.battery_level = SYS_STATUS.battery_remaining
        self.voltage_level = SYS_STATUS.voltage_battery
        self.current_battery = SYS_STATUS.current_battery

    def idle_task(self):
        '''time motor events, track battery usage, and time sensor readings'''
        now = time()
        if self.module('rc').override_period.trigger():
            if (self.module('rc').override != [1500] * 16 or
                self.module('rc').override != self.module('rc').last_override or
                self.module('rc').override_counter > 0):
                self.module('rc').last_override = self.module('rc').override[:]
                self.module('rc').send_rc_override()
                if self.module('rc').override_counter > 0:
                    self.module('rc').override_counter -= 1
        # extremely time critical, as the code must send the next 'rc' cmd before the 3 second disarm timeout
        if self.end_time <= time():
            self.module('rc').stop()  # this resets the disarm timer
            # * Code between asterisks must execute in under 3 seconds to keep auv armed
            try:
                command=self.command_queue.popleft()
                self.cmd_move(command)
                self.end_time=time() + command[2]
                self.write_to_battery.append("uJoules: %s, Run time: %s, Time: %s \n" % (self.ujoules, self.motor_run_time, strftime("%H:%M:%S")))
                self.motor_run_time=command[2]
                self.ujoules=0
            # *
            except IndexError:
                self.end_time = time() + 1
                with open("/home/pi/motor_battery.txt", "a+") as f:
                    f.write(''.join(self.write_to_battery))
                self.write_to_battery = []
        if now - self.last_batt >= 1:
            self.last_batt=now
            self.ujoules += self.batt_info()
        if now - self.last_sample > 1:
            self.last_sample=now
            self.sample()
        self.mav.set_mode_manual()  # keeps auv armed

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()
        if mtype == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.gps_update(m)

        if mtype == 'SCALED_PRESSURE3':
            self.psensor_update(m)

        if mtype == 'SCALED_PRESSURE':
            self.dsensor_update(m)

        if mtype == "SYS_STATUS":
            self.battery_update(m)

        if mtype in ['WAYPOINT_COUNT', 'MISSION_COUNT']:
            if self.wp_op is None:
                self.console.error("No waypoint load started")
            else:
                self.wploader.clear()
                self.wploader.expected_count = m.count
                self.console.writeln("Requesting %u waypoints t=%s now=%s" % (m.count,
                                                                              time.asctime(time.localtime(m._timestamp)), time.asctime()))
                self.send_wp_requests()

        elif mtype in ['WAYPOINT', 'MISSION_ITEM'] and self.wp_op is not None:
            if m.seq < self.wploader.count():
                # print("DUPLICATE %u" % m.seq)
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
                if self.logdir is not None:
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
                    self.say("waypoint %u" % m.seq, priority='message')

        elif mtype == "MISSION_ITEM_REACHED":
            wp = self.wploader.wp(m.seq)
            if wp is None:
                # should we spit out a warning?!
                # self.say("No waypoints")
                self.next_wp = None
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
            if self.fence_manager.present is False and present is True:
                self.say("fence present")
            elif self.fence_manager.present is True and present is False:
                self.say("fence removed")
            self.present = present

            enabled = ((m.onboard_control_sensors_enabled & bits) == bits)
            if self.fence_manager.enabled is False and enabled is True:
                self.say("fence enabled")
            elif self.fence_manager.enabled is True and enabled is False:
                self.say("fence disabled")
            self.fence_manager.enabled = enabled

            healthy = ((m.onboard_control_sensors_health & bits) == bits)
            if self.fence_manager.healthy is False and healthy is True:
                self.say("fence OK")
            elif self.fence_manager.healthy is True and healthy is False:
                self.say("fence breach")
            self.fence_manager.healthy = healthy

            # console output for fence:
            if self.fence_manager.enabled is False:
                self.fence_manager.console.set_status('Fence', 'FEN', row=0, fg='grey')
            elif self.fence_manager.enabled is True and self.fence_manager.healthy is True:
                self.console.set_status('Fence', 'FEN', row=0, fg='green')
            elif self.fence_manager.enabled is True and self.fence_manager.healthy is False:
                self.console.set_status('Fence', 'FEN', row=0, fg='red')


class HardwareError(EnvironmentError):
    def __init__(self, args):
        self.errno = int(args[0])
        self.message = str(args[1])


class motor_event(object):
    '''a class for fixed frequency events'''
    def __init__(self, seconds):
        self.seconds = seconds
        self.curr_time = time()
        self.final_time = self.curr_time + seconds

    def force(self):
        '''force immediate triggering'''
        self.curr_time = 0

    def trigger(self):
        ''' True if we should trigger now'''
        tnow = time()

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
