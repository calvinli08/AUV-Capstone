#!/usr/bin/env python

'''
This is the Python implementation of the sparse/dense traversal algorithm outlined in:

B. Chen, P. Pandey, and D. Pompili. "A distributed adaptive sampling solution using autonomous underwater vehicles." in IFAC Proceedings Volumes 45, no. 27 pp. 352-356. 2012.

It directs the AUV to travel in a sparse lawnmower pattern across the mission area while sampling for pollution.
If the AUV detects a pollution concentration greater than a certain threshold during the sparse traversal
it will automatically switch into a dense traversal mode, performing denser lawnmower sweeps in the direction of the
pollution. Afterwards, it will return to sparse sweeping.
Users can also enter Points of Interest to the AUV, which will be examined by the AUV using dense traverses
after completing the whole sparse traversal. POI will be visited in the order given by a nearest neighbor algorithm.
Waypoints will be fed into the AUV class upon __init__ from file where they're stored
'''

#TODO figue out how to spin popellers, get heading underwater, check logical flow, make sure there are no background processes obstructing this code, read mavutils, try to replace as many of these operations with mavutils

from pymavlink import mavutil
from collections import deque
import os, time, math
from MAVProxy.modules import mavproxy_battery as bm
from MAVProxy.modules import mavproxy_DGPS as dgps
from MAVProxy.modules.lib import mp_module, mp_util
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules import mavproxy_wp as wp
from MAVProxy.modules import mavproxy_rc as rc

class AUVModule(mp_module.MPModule):
    #__init__
    def __init__(self):
        super(AUVModules, self).__init__(mpstate, "AUV", "AUV sparse/dense traversal algorithm")
        radctrl = rc()
        wypt = wp()
        batm = bm()
        mav = mavutil.mavfile()
        self.battery = batm.percentage()
        #update using the dataflash logs
        self.velocity = 0
        self.time = time.clock()
        self.surfaced = True
        mav.set_mode_apm(9)
        #GPS dependent values
        self.location = mav.location()
        self.home = [self.location.lng, self.location.lat]
        self.waypoints = deque(wypt.load_waypoints_dive('/home/pi/waypoints.txt'))
        wp = self.waypoints.popleft().rsplit()
        self.next_wp = [float(wp[3]), float(wp[2])] #lng,lat
        self.heading = self.location.heading #degrees relative to Magnetic North
        self.distance = mp_util.gps_distance(self.location.lat, self.location.lng, self.next_wp[1], self.next_wp[0]) #meters
        self.intended_heading = self.gps_bearing(self.location.lat, self.location.lng, self.next_wp[1], self.next_wp[0] )#degrees relative to present heading
        #need a current sensor for this. Let's try to find average current info online and hardcode it for now.
        self.current = 0 #m/s
        #MAVProxy parameters
        self.status_callcount = 0
        self.boredom_interval = 1 # seconds
        self.last_bored = time.clock()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = True

        self.AUVModule_settings = mp_settings.MPSettings([ ('verbose', bool, True), ])
        self.add_command('auv', self.cmd_auv, "AUV module", ['status','set (LOGSETTING)', 'start', 'reboot'])

        #set manual mode
        mav.set_mode_manual()
        radctrl.set_mode_manual()
        #wait for motors to be armed
        mav.motors_armed_wait()
        #set the apm mav_type
        mav.mode_mapping()
'''
mode_mapping_sub = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'VELHOLD',
    6: 'RTL',
    7: 'CIRCLE',
    9: 'SURFACE',
    10: 'OF_LOITER',
    11: 'DRIFT',
    13: 'TRANSECT',
    14: 'FLIP',
    15: 'AUTOTUNE',
    16: 'POSHOLD',
    17: 'BRAKE',
    18: 'THROW',
    19: 'MANUAL',
    }

'''

    def usage(self):
        '''show help on command line options'''
        return "Usage: auv <status|set|start|reboot>"

    def cmd_auv(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.example_settings.command(args[1:])
        elif args[0] == "start":
            self.predive()
        elif args[0] == "reboot":
            mav.reboot_autopilot()
        else:
            print self.usage()

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.clock() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" % {"status_callcount": self.status_callcount, "packets_mytarget": self.packets_mytarget,"packets_othertarget": self.packets_othertarget,})

    def boredom_message(self):
        if self.example_settings.verbose:
            return ("Waiting for waypoints. To enter waypoints, edit the /home/pi/waypoints.txt file.\n If the AUV is low on battery it will not dive; if the AUV returned to the home point before a mission completed, that signifies that it is low on battery and the battery needs to be changed.\n Furthermore, an inability to gain a proper compass heading will cause the AUV to remain immobile.\n")
        return ("Waiting for waypoints. Have you checked the battery and compass?\n")

    def idle_task(self):
        '''
        Called rapidly by mavproxy.
        Functions and actions that must be performed repeatedly:
        update()
        '''
        now = time.clock()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message)
            self.update()

    #updates all the navigational parameters. Essentially __init__.
    def update(self):
        self.battery = batm.percentage()
        self.heading =
        self.velocity =
        self.time = time.clock()
        self.current = 1

    #Update the AUV's current GPS location, and get the cardinal direction of the next waypoint, and distance to it.
    def gps_update(self, next_wp):
        #No point in checking it underwater where there is no GPS fix. That might also feed corrupted data to the module, so avoid calling this underwater.
        if self.surfaced == True:
            self.location = mav.location()
            self.distance = mp_util.gps_distance(self.location.lat, self.location.lng, next_wp[1], next_wp[0])
            self.heading = self.location.heading
            self.intended_heading = self.gps_bearing(self.location.lat, self.location.lng, next_wp[1], next_wp[0])
        else:
            return

    #Maneuvers the AUV while on the surface back towards the home point for maintenance, etc
    def go_home(self):
        #Traverse to home waypoint
        while self.location != self.home:
            self.update()
            self.gps_update(self.home)
            self.orient_heading(self.intended_heading)
            radctrl.set_override([0 0 0 0 0 0 0 0 0])#spin forward propellers

    #get the next waypoint
    def get_next_wp(self):
        #Get the current GPS coordinate
        self.gps_update(self.next_wp)

        #Set the next waypoint if the last waypoint was reached
        #If no more waypoints, go home
        if not self.waypoints:
           return False
        elif self.location == self.next_wp:
           self.next_wp = self.waypoints.popleft()
           return True
        return True

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

    #Dives to a set depth, turns on the lights, and travels at a constant velocity to target
    def dive(self, dive_point, destination, distance, depth = 1, heading, current = 1):
        #call the mavproxy command to spin motors down till the required depth
        for i in range(0, depth, 0.5):
            radctrl.cmd_rc([5, 1800])#down motors
            radctrl.cmd_rc[6, 1200])
        self.surfaced = False

    #Determines whether to surface
    def surface(self):
        '''
        BATTERY is measured as a percentage
        TIME is in minutes
        VELOCITY is in meters/second

        Surface check does not allow the AUV loiter on the surface if it has enough resources to travel to the
        next waypoint.
        The only time the AUV loiters is when it has low battery, and returns home instead; it achieves this by calling predive().
        '''
        self.update()
        if self.battery <= 35.0 or self.time >= 180 or self.velocity < 0.5:
        mav.set_servo(channel, 1700)#surface command, spin vertical motors
        mav.set_mode_apm(9)#set mode to surfaced
        self.surfaced = True
        return

    #Check that the AUV is facing the correct cardinal direction and correct it if necessary
    def orient_heading(self, intended_heading):
        self.update()
        while intended_heading > 5:
            #Correct the error
            if intended_heading > 0:
                radctrl.set_override([1800 0 0 1800 0 0 0 0])#change yaw ccw
            else:
                radctrl.set_override([0 1800 1800 0 0 0 0 0])#change yaw cw
        else:
            #once correction is complete, stop motors
            radctrl.set_override([0 0 0 0 0 0 0 0])

    #Sample pollution with the sensors
    def sample(self):
        #read from serial in

        return #pollution level as a float

    #traverse
    def traverse_and_sample(self, time):
        while mav.motors_armed():
            #Is this how it's done? Pixhawk docs say to not power servo directly, so this may be dangerous
            #perhaps i am supposed to use another functions for motors? or power esc?
            #if use servos: find map of servos and motors, also figure out how much pwm to send
            #http://docs.bluerobotics.com/thrusters/t200/#performance-charts PWM VS THRUST CHART
            #let's try 1800Hz@12V as default forward / 1200Hz@12V as default reverse and adjust from there
            mav.set_servo(channel,1800)#move forward at 1 m/s
            mav.set_servo(channel depends on current, 1800)#spin thrusters in opposite direction of current
        return self.sample()

    #underwater sparse traverse function
    #The core of the AUV class: directs the AUV to travel from it's current location to a waypoint
    def underwater_traverse(self, start, end, distance, heading, current = 1):
        '''
        Tell the AUV to move at a constant speed in the heading direction while accounting for current.
        In order to determine when to surface, calculate the estimated time to travel to destination, and then surface once that time is reached.
        Latency is the amount of extra time it takes to calculate the while loop conditions and to execute it. This latency is added to END_TIME, because due to the extra time it takes to calculate conditions between telling the AUV to move, the estimated travel time to the destination will increase.
        '''
        travel_time = distance
        start_time = time.clock()
        latency =
        end_time = start_time + travel_time + latency
        '''Measure the run times and order of how this code segment runs'''
        while self.surface() is False and self.orient_heading(heading):
            if end_time - time.clock() >= 0:
                if self.traverse() > threshold:
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
        Calculate the cardinal directions that are orthogonal to HEADING on both sides.
        Determine the total width and length of the traversal area.
        '''
        #Grab the start time
        start_time = time.clock()
        #Calculate orthogonal cardinal directions from HEADING
        left_direction = heading#rotate 90 deg ccw
        right_direction = heading#rotate 90 deg cw

        #Orient vehicle in leftwards orthogonal cardinal direction
        self.orient_heading(left_direction)
        for i in range(distance/2):
            self.traverse(time)


        return time.clock() - start_time

    def run_mission(self):
        self.get_next_wp()
        self.update()
        self.gps_update(self.next_wp)
        if self.predive():
            self.dive(self.location, self.next_wp, distance, depth = 1, self.intended_heading, current = 1)



    #handle a mavlink packet
    def mavlink_packet(self, m):

def init(mpstate):
    return AUVModule(mpstate)
