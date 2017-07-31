#!/usr/bin/env python

import sys
import numpy
import errno
import json
import curses
from pymavlink import mavutil, mavwp
from time import strftime, time
from collections import deque
from math import sqrt, pow
from os import system
from re import match, search, compile

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules import SerialReader


class AUVModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(AUVModule, self).__init__(mpstate, "auto", "Autonomous navigation module")

        '''Navigational information'''
        self.waypoints = deque()
        self.next_wp = None  # lat,lng
        self.wp_perimeter = ()
        self.reached_wp = False
        self.offset_from_intended_heading = 0
        self.pollution_array = numpy.zeros([100, 100], numpy.dtype(object), 'C')  # initialize later
        self.loops = 0
        self.xy = {'x': 0, 'y': 0}  # x,y
        self.y = True
        self.dense = False
        self.start_time = 0
        self.home = {'lat': 0, 'lng': 0}

        '''GPS output'''
        self.gps_disp = curses.newwin(7, 70, 4, 5)
        self.gps_disp.keypad(True)
        self.gps_disp.clear()
        curses.noecho()
        curses.cbreak()

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

        '''Sampling'''
        self.last_sample = time()
        self.sensor_reader = SerialReader.SerialReader()

        ''' Commands for operating the module from the MAVProxy CLI'''
        self.add_command('auto', self.cmd_auto, "Autonomous sampling traversal", ['home', 'test', 'mission', 'setfence'])

        '''low-level pymavlink functionality'''
        self.wp_loader = mavwp.MAVWPLoader(self.target_system, self.target_component)

        '''Failsafe checks'''
        self.time_at_last_heartbeat = time()
        self.servo_output_raw = []
        self.rc_channels_raw = []
        self.rc_regexp = compile('chan[1-6]_raw')
        self.servo_regexp = compile('servo[1-6]_raw')
        self.mav_state_critical = False
        self.mav_state_emergency = False

    def usage(self):
        '''show help on command line options'''
        return "Usage: auto <test|setfence|mission>"

    def cmd_auto(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "mission":
            try:
                self.load_waypoints('/home/pi/waypoints.plan')
            except FileError:
                print FileError.message
                return
            return self.run()
        elif args[0] == "setfence":
            self.load_geofence_points('home/pi/fence.txt')
            self.calculate_geofence_edge_lengths()
            return
        elif args[0] == "test":
            print self.test1()
        elif args[0] == "home":
            self.home['lat'] = args[1]  # lat
            self.home['lng'] = args[2]  # lng
            return
        else:
            print self.usage()
        return

    def load_waypoints(self, filename):
        with open(filename, "r") as f:
            waypoints_dict = json.load(f)
            waypoints_list = waypoints_dict['mission']['items']
        for wp in waypoints_list:
            self.waypoints.append(tuple(wp['coordinate'][0:2]))

        self.next_wp = self.waypoints.popleft()
        self.wp_perimeter = (self.next_wp[0] + 0.0005, self.next_wp[1] + 0.0005, self.next_wp[0] - 0.0005, self.next_wp[1] - 0.0005)

        if self.next_wp is None:
            raise FileError("Waypoint loading unsuccessful. Please check that the waypoint file is populated.\n")
        return

    def calculate_geofence_edge_lengths(self):
        '''calculate length of geofence rectangle sides'''
        points = distance_between_points = p = []
        print(1)
        with open('/home/pi/fence.txt', "r") as f:
            for line in f:
                p = line.strip().split(',')
                for i in range(len(p)):
                    p[i] = float(p[i])
                points.append(p)
        for x in range(len(points)):
            if x+1 is len(points):
                distance_between_points.append(mp_util.gps_distance(points[x][0], points[x][1], points[0][0], points[0][1]))
                print(mp_util.gps_distance(points[x][0], points[x][1], points[0][0], points[0][1]))
            else:
                distance_between_points.append(mp_util.gps_distance(points[x][0], points[x][1], points[x+1][0], points[x+1][1]))

        return (min(distance_between_points), max(distance_between_points))  # width, length

    def load_geofence_points(self, filename):
        return self.modules('fence').cmd_fence(['load', filename])

    def track_xy(self, y=False):
        if y:
            self.xy['y'] += 1
        elif self.loops % 2 == 0:
            self.xy['x'] += 1
        elif self.loops % 2 == 1:
            self.xy['x'] = len(self.pollution_array)
            self.xy['x'] -= sign
        return

    '''unit test delete later '''
    def test1(self):
        '''yaw motor test'''

        '''yaw clockwise for 3 seconds'''
        self.command_queue.append(["y", 1540, 4])

        '''yaw ccw for 3 seconds'''
        self.command_queue.append(["y", 1450, 4])

    '''Performs pre-dive information gathering and checks'''
    def predive_check(self):
        if not self.next_wp:
            raise ValueError('self.next_wp is None')
        if self.battery_percentage() >= 60.0:
            return
        elif 55.0 <= self.battery_percentage() < 60.0:
            return
        else:
            #raise HardwareError([1, 'Insufficient Battery'])
            return

    def run(self):
        '''Main command loop for the mission'''
        try:

            print("Traveling to: %s\n" % (str(self.next_wp)))

            self.predive_check()  # checks that auv is ready to dive, throws exception if not

            # Distance to the next waypoint
            self.distance_to_waypoint = mp_util.gps_distance(self.lat, self.lon,
                                                             self.next_wp[0], self.next_wp[1])

            # Offset in degrees from the direction of the next waypoint
            self.offset_from_intended_heading = mp_util.gps_bearing(self.lat, self.lon,
                                                                    self.next_wp[0], self.next_wp[1])

            # Spins the AUV towards the correct direction
            self.orient_heading(self.offset_from_intended_heading)

            # Sets the size of the pollution array
            #array_edges = self.calculate_geofence_edge_lengths()
            self.pollution_array = numpy.zeros([150, 150], numpy.dtype(object), 'C')  # each square meter is a point

            # self.dive()

            self.underwater_traverse(self.distance_to_waypoint)

            # self.surface()

            # Tells this function to run again, this implements the loop functionality
            self.continue_mission()

        except HardwareError:
            print(HardwareError.message)
            print('Hardware Error')
            return self.go_home()

        except ValueError:
            print(ValueError)
            print('Value Error')
            return self.go_home()

        return numpy.save('/home/pi/pollution_array.npy', self.pollution_array)

    # traverse
    # assuming: one second = one meter, 2 seconds delay
    def traverse(self, time=3):
        return self.command_queue.append(["f", 1600, time])

    def surface(self, time=5):
        return self.command_queue.append(["z", 1650, time])

    def dive(self, time=3):
        return self.command_queue.append(["z", 1450, time])

    def orient_heading(self, offset_from_intended_heading, pwm=1550):
        diff = abs(pwm - 1500)
        ccw_pwm = 1500 - diff
        cw_pwm = 1500 + diff

        if offset_from_intended_heading > 0:
            return self.command_queue.append(["y", ccw_pwm, 20])
        else:
            return self.command_queue.append(["y", cw_pwm, 20])


    def go_home(self):
        '''returns to home coordinate'''
        print ('Returning Home\n')
        self.module('rc').override = [1500, 1700, 1500, 1500, 1500, 1500, 1500, 1500]
        self.module('rc').send_rc_override()
        sleep(2)

        # Distance to the next waypoint
        self.distance_to_waypoint = mp_util.gps_distance(self.lat, self.lon, self.home[lat], self.home[lng])

        # Offset in degrees from the direction of the next waypoint
        self.offset_from_intended_heading = mp_util.gps_bearing(self.lat, self.lon, self.home[lat], self.home[lng])

        # Spins the AUV towards the correct direction
        self.orient_heading(self.offset_from_intended_heading)

        self.module('rc').stop()
        self.command_queue.clear()
        self.end_time = 0
        self.command_queue.append(["f", 1650, self.distance_to_waypoint])

    def continue_mission(self):
        return self.command_queue.append(["continue_mission", 1500, 0])

    # args = [direction, pwm, seconds]
    # roll - 3
    # z - 2
    # yaw - 4
    # forward - 5
    # lateral - 6
    def cmd_move(self, args, y=False):
        if len(args) != 3:
            return "Usage: move <f|l|z|roll|yaw> pwm"
        elif args[0] == "f":  # forward
            print('forward')
            self.module('rc').override[4] = args[1]
            self.module('rc').send_rc_override()
            return self.track_xy(args[1])
        elif args[0] == "l":  # lateral
            # This is how the joystick module does it
            self.module('rc').override[5] = args[1]
            return self.module('rc').send_rc_override()
        elif args[0] == "z":  # dive/surface
            self.module('rc').override[1] = args[1]
            return self.module('rc').send_rc_override()
        elif args[0] == "r":  # roll
            self.module('rc').override[2] = args[1]
            return self.module('rc').send_rc_override()
        elif args[0] == "y":  # yaw
            print('yaw')
            self.module('rc').override[3] = args[1]
            return self.module('rc').send_rc_override()
        elif args[0] == "end_dense":
            self.dense = False
            self.module('rc').override[4] = args[1]
            return self.module('rc').send_rc_override()
        elif args[0] == "continue_mission":
            self.reached_wp = False
            return self.run()
        else:
            return "Usage: move <f|l|z|r|y> pwm seconds"

    '''underwater sparse traverse function'''
    def underwater_traverse(self, distance, current=1):
        self.start_time = time()
        if distance == 1:
            self.loops += 1
        '''Measure the run times and order of how this code segment runs'''
        return self.traverse(distance+1)

    ''' dense traverse function that is called when pollution is above a threshold'''
    def dense_traverse(self, forward_increment=3, pwm=1550, forward_distance_to_edge=10, loop_number=3, forward_travel_distance=5, sideways_distance=4, current=0):

        self.module('rc').stop()
        self.command_queue.clear()
        self.end_time = 0
        self.dense = True

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

        remaining_distance = end_time - forward_travel_distance - elapsed_time
        end_time = int(time()) + remaining_distance + 1

        return forward_travel_distance

    def batt_info(self):
        return float(self.current_battery) * float(self.voltage_level)  # micro-watts

    def sample(self):
        do = float(self.sensor_reader.read("2").rstrip())
        cond = float(self.sensor_reader.read("3").rstrip())
        temp = self.temp_sensor[2]

        with open("/home/pi/sensor_battery.txt", "a+") as f:
            f.write("DO: %s, Cond: %s, Temp: %s, Lat: %s, Long: %s, uWatts: %s, Time: %s \n" % (do, cond, temp, self.lat, self.lon, self.batt_info(), strftime("%H:%M:%S")))  # DO, Conductivity, Temperature, Lat, Lng, microWatts, time

        self.pollution_array[self.xy['x'], self.xy['y']] = (do, cond, temp)
        numpy.save('/home/pi/pollution_array.npy', self.pollution_array)

        bound_check = (do >= 14.0, do <= 5.0, cond >= 800.0, temp >= 3000.0, temp <= 1000.0)

        print("%s > %s\n" % (str((do, cond, temp)), str(bound_check)))

        return bound_check

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
        '''Four decimal places of precision are required to notice changes in position'''
        if (self.wp_perimeter[2], self.wp_perimeter[3]) <= (self.lat, self.lon) and (self.lat, self.lon) <= (self.wp_perimeter[0], self.wp_perimeter[1]):
            self.next_wp = self.waypoints.popleft()
            self.wp_perimeter = (self.next_wp[0] + 0.0005, self.next_wp[1] + 0.0005, self.next_wp[0] - 0.0005, self.next_wp[1] - 0.0005)
            self.reached_wp = True

        self.gps_disp.addstr(4, 7, "GPS: (lat: %s, long: %s, hdg: %s)\n" %(self.lat, self.long, self.hdg))
        self.gps_disp.refresh()

        self.reached_wp = False

    def rc_update(self, RC_CHANNELS_RAW):
        #self.rc_channels_raw = [1800 < RC_CHANNELS_RAW[key] < 1200 for key in RC_CHANNELS_RAW.iteritems() if self.rc_regexp.match(key)]
        return

    def servo_update(self, SERVO_OUTPUT_RAW):
        #self.servo_output_raw = [1800 < SERVO_OUTPUT_RAW[key] < 1200 for key in SERVO_OUTPUT_RAW.iteritems() if self.servo_regexp.match(key)]
        return

    def battery_update(self, SYS_STATUS):
        '''update battery level'''
        self.voltage_level = SYS_STATUS.voltage_battery
        self.current_battery = SYS_STATUS.current_battery
        return

    def battery_percentage(self):
        voltage = self.voltage_level/1000
        if 16 <= voltage < 17:
            return 75
        elif 15 <= voltage < 16:
            return 50
        elif 14 <= voltage < 15:
            return 40
        elif 13 <= voltage < 14:
            return 30

    def hardware_check(self):
        '''Checks that hardware is well functioning'''
        if any(self.servo_output_raw) or any(self.rc_channels_raw):
            raise PWMError('PWM values out of bounds, disarming')
        elif self.battery_percentage() < 40:
            print(2)
            raise HardwareError([2, 'Low Battery'])
        elif time() - self.time_at_last_heartbeat > 300:
            print(3)
            raise HardwareError([3, 'Lost connection to Pixhawk'])
        elif self.mav_state_critical:
            print(4)
            raise HardwareError([4, 'Critical Condition'])
        elif self.mav_state_emergency:
            print(0)
            raise HardwareError([0, 'Irrecoverable Electrical Failure'])
        return

    def surface_and_disarm(self, message):
        print(message)
        self.module('rc').override = [1500, 1700, 1500, 1500, 1500, 1500, 1500, 1500]
        self.module('rc').send_rc_override()
        sleep(6)
        return self.module('arm').disarm('force')

    def idle_task(self):
        '''time motor events, track battery usage, monitor system, and time sensor readings'''
        now = time()

        # try:
        #     self.hardware_check()
        #
        # except PWMError:
        #     print(PWMError.message)
        #     print ('disarminggggggggggggg')
        #     self.module('arm').disarm('force')
        #
        # except HardwareError:
        #     print('errrorrrrr')
        #     if HardwareError.errno is 2:
        #         print(2)
        #         #print(HardwareError.message)
        #         #self.go_home()
        #     elif HardwareError.errno is 3:
        #         print(3)
        #         #self.surface_and_disarm(HardwareError.message)
        #     elif HardwareError.errno is 4:
        #         print(4)
        #         #self.surface_and_disarm(HardwareError.message)
        #     elif HardwareError.errno is 0:
                #print(0)
                #self.surface_and_disarm(HardwareError.message)  # try to surface
                #self.mav.reboot_autopilot()
                #system('sudo reboot now')

        if self.module('rc').override_period.trigger():
            if (self.module('rc').override != [1500] * 16 or
                self.module('rc').override != self.module('rc').last_override or
                self.module('rc').override_counter > 0):
                self.module('rc').last_override = self.module('rc').override[:]
                self.module('rc').send_rc_override()
                if self.module('rc').override_counter > 0:
                    self.module('rc').override_counter -= 1

        if mp_util.gps_bearing(self.lat, self.lon, self.next_wp[0], self.next_wp[1]) <= 2:
             self.module('rc').stop()
             self.end_time = 0

        if self.reached_wp:
            self.module('rc').stop()
            self.write_to_battery.append("uJoules: %s, Run time: %s, Time: %s \n" % (self.ujoules, self.motor_run_time, strftime("%H:%M:%S")))
            self.cmd_move(["continue_mission", 1500, 0])

        # extremely time critical, as the code must send the next 'rc' cmd before the 3 second disarm timeout
        if self.end_time <= time():
            self.module('rc').stop()  # this resets the disarm timer
            # * Code between asterisks must execute in under 3 seconds to keep auv armed
            self.write_to_battery.append("uJoules: %s, Run time: %s, Time: %s \n" % (self.ujoules, self.motor_run_time, strftime("%H:%M:%S")))
            try:
                command=self.command_queue.popleft()
                self.cmd_move(command)
                self.end_time = time() + command[2]
                self.motor_run_time = command[2]
                self.ujoules = 0
            # *
            except IndexError:
                self.end_time = time() + 1
                with open("/home/pi/motor_battery.txt", "a+") as f:
                    f.write(''.join(self.write_to_battery))
                self.write_to_battery = []

        if now - self.last_batt >= 1:
            self.last_batt = now
            self.ujoules += self.batt_info()

        if now - self.last_sample > 1:
            self.last_sample = now
            # * Code between asterisks must execute in under 3 seconds to keep auv armed
            if any(self.sample()) and self.dense is False:
                print('Pollution information exceeding threshold, starting dense traversal\n')
                self.command_queue.append(["end_dense", 1600, (self.end_time - self.dense_traverse() - (time() - self.start_time))])
                self.surface()
        # *
        return

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()

        if mtype == 'HEARTBEAT':
            self.time_at_last_heartbeat = time()

        elif mtype == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.gps_update(m)

        elif mtype == 'SCALED_PRESSURE3':
            self.psensor_update(m)

        elif mtype == 'SCALED_PRESSURE':
            self.dsensor_update(m)

        elif mtype == "SYS_STATUS":
            self.battery_update(m)

        elif mtype == 'RC_CHANNELS_RAW':
            self.rc_update(m)

        elif mtype == 'SERVO_OUTPUT_RAW':
            self.servo_update(m)

        elif mtype == 'MAV_STATE_CRITICAL':
            self.mav_state_critical = True

        elif mtype == 'MAV_STATE_EMERGENCY':
            self.mav_state_emergency = True

        elif mtype in ['WAYPOINT_COUNT', 'MISSION_COUNT']:
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
            wp = self.module('wp').wploader.wp(m.seq)
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
            self.module('fence').last_fence_breach = m.breach_time
            self.module('fence').last_fence_status = m.breach_status
        elif m.get_type() in ['SYS_STATUS']:
            bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

            present = ((m.onboard_control_sensors_present & bits) == bits)
            if self.module('fence').present is False and present is True:
                self.say("fence present")
            elif self.module('fence').present is True and present is False:
                self.say("fence removed")
            self.present = present

            enabled = ((m.onboard_control_sensors_enabled & bits) == bits)
            if self.module('fence').enabled is False and enabled is True:
                self.say("fence enabled")
            elif self.module('fence').enabled is True and enabled is False:
                self.say("fence disabled")
            self.module('fence').enabled = enabled

            healthy = ((m.onboard_control_sensors_health & bits) == bits)
            if self.module('fence').healthy is False and healthy is True:
                self.say("fence OK")
            elif self.module('fence').healthy is True and healthy is False:
                self.say("fence breach")
            self.module('fence').healthy = healthy

            # console output for fence:
            if self.module('fence').enabled is False:
                self.module('fence').console.set_status('Fence', 'FEN', row=0, fg='grey')
            elif self.module('fence').enabled is True and self.module('fence').healthy is True:
                self.console.set_status('Fence', 'FEN', row=0, fg='green')
            elif self.module('fence').enabled is True and self.module('fence').healthy is False:
                self.console.set_status('Fence', 'FEN', row=0, fg='red')

            return


class HardwareError(EnvironmentError):
    '''
    errno:
    0 - Irrecoverable Electrical Failure
    1 - Insufficient Battery
    2 - Low Battery
    3 - Lost Connection to Pixhawk
    4 - Motor Failure
    '''
    def __init__(self, args):
        self.errno = int(args[0])
        self.message = str(args[1])


class FileError(Exception):
    def __init__(self, message):
        self.message = message


class PWMError(EnvironmentError):
    def __init__(self, message):
        self.message = message


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
