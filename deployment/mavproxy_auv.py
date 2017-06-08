#!/usr/bin/env python

'''
This is the Python implementation of the sparse/dense traversal algorithm outlined in
Pompili et al. 2012
It directs the AUV to travel in a sparse lawnmower pattern across the mission area while sampling for pollution.
If the AUV detects a pollution concentration greater than a certain threshold during the sparse traversal
it will automatically switch into a dense traversal mode, performing denser lawnmower sweeps in the direction of the
pollution. Afterwards, it will return to sparse sweeping.
Users can also enter Points of Interest to the AUV, which will be examined by the AUV using dense traverses
after completing the whole sparse traversal. POI will be visited in the order given by a nearest neighbor algorithm.
Waypoints will be fed into the AUV class upon __init__ from file where they're stored
'''

from pymavlink import mavutil as mav
from collections import deque
import os, time
from MAVProxy.modules import mavproxy_battery as bm
from MAVProxy.modules import mavproxy_DGPS as dgps
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules import mavproxy_wp as wp

class AUVModule(mp_module.MPModule):
    #__init__
    def __init__(self):
        super(AUVModules, self).__init__(mpstate, "AUV", "AUV sparse/dense traversal algorithm")
        with open('/home/uav/dataflash/dataflash.txt',"r") as log:
            self.battery = bm.BatteryModule.percentage()
            self.velocity = 0
            self.magX =
            self.magY =
            self.magZ =
            #update using the dataflash logs
            self.time = time.time()
            self.surfaced = True
            #GPS dependent values
            self.location =
            self.waypoints = deque(wp.WPModule.load_waypoints_dive('/home/pi/waypoints.txt'))
            self.distance = 0
            self.bearing =
            self.intended_bearing =
            #need a current sensor for this
            self.current =


    #updates all the navigational parameters. Essentially __init__.
    def update(self):
        with open('''path to log''',"r") as log:
            self.battery = bm.BatteryModule.percentage()
            self.bearing = log.readline()
            self.velocity = log.readline()
            self.magX =
            self.magY =
            self.magZ =
            self.time =
            self.current =

    #Update the AUV's current GPS location and the next waypoint's location. Also get the cardinal direction of the next waypoint, and distance to it.
    def gps_update(self):
        self.location =
        self.distance = #difference between waypoint and location
        self.bearing =
        self.intended_bearing =

    #Maneuvers the AUV while on the surface back towards the home point for maintenance, etc
    def home(self):
        #Traverse to home waypoint
        while location != home:
            self.update()
            self.gps_update()
            self.bearing_check()
            mav.#spin forward propellers


    #Performs pre-dive information gathering and checks
    def predive(self):
        #Get the current GPS coordinate
        self.gps_update()

        #Set the next waypoint if the last waypoint was reached
        if self.location == next_wp:
            next_wp = self.waypoints.popleft()

        #Detect the river's present current
        #Is there a module for this?
        #Check if stats are fit for diving i.e. is there enough battery, mission area, etc
        self.update()
        #Perform arm_throttle checks and any QGroundControl checks. Also, set a home waypoint.

        qgc_checks = #boolean
        if self.battery >= 60.0 and qgc_checks:
            #Orient the vehicle in the correct cardinal direction
            self.bearing_check(self.intended_bearing)
            #Calculate distance between dive_point and next_wp
            distance =
            #Since there is enough battery, dive
            self.dive(self.location, next_wp, 1, self.intended_bearing, 1)
            return True
        elif 35.0 <= self.battery < 60.0 and qgc_checks:
            #If the distance is within coverable distance, dive
            if self.battery*time_multiplier - (distance / self.velocity) > 2 '''seconds''':
                #Orient the vehicle in the correct cardinal direction
                self.bearing_check(self.intended_bearing)
                #Calculate distance between dive_point and next_wp
                distance =
                #Dive
                self.dive(self.location, next_wp, 1, self.intended_bearing, 1)
                return True
            else:
                #Return to the home point
                self.home()
                return False
        else:
            #Return to the home point
            self.home()
            return False

    #Dives to a set depth, turns on the lights, and travels at a constant velocity to target
    def dive(self, dive_point, destination, depth = 1, bearing, current = 1):
        #call the mavproxy command to spin motors down till the required depth
        for i in range(0, depth, 0.5):
            mav.
        self.surfaced = False
        self.underwater_traverse(dive_point, destination, '''distance''', bearing, current)

    #Determines whether to surface
    def surface(self):
        '''
        BATTERY is measured as a percentage
        TIME is in minutes
        VELOCITY is in meters/second

        Surface check is not meant to have the AUV loiter on the surface if it has enough resources to travel to the
        next waypoint.
        The only time the AUV loiters is when it has low battery, and returns home instead.
        '''
        self.update()
        if self.battery <= 35.0 or self.time >= 180 or self.velocity < 0.5:
            mav.#surface command, spin vertical motors
            self.surfaced = True
            self.predive()
            return True
        else:
            return False

    #Check that the AUV is facing the correct cardinal direction and correct it if necessary
    def bearing_check(self, intended_bearing):
        self.update()
        #Calculate offset from intended bearing
        xOffset = self.magX - intended_bearing['magX']
        yOffset = self.magY - intended_bearing['magY']
        zOffset = self.magZ - intended_bearing['magZ']

        if xOffset > 5 or yOffset > 5 or zOffset > 5:
            #Correct the error
            mav.#change yaw
            mav.#change pitch
            mav.#change roll
        else:
            break

    #Sample pollution with the sensors
    def sample(self):
        #read from serial in
        return

    #traverse
    def traverse(self, time):
        
        mav.#move forward at 1 m/s
        mav.#spin thrusters in opposite direction of current

    #underwater sparse traverse function
    #The core of the AUV class: directs the AUV to travel from it's current location to a waypoint
    def underwater_traverse(self, start, end, distance, bearing, current = 1):
        '''
        Tell the AUV to move at a constant speed in the bearing direction while accounting for current.
        In order to determine when to surface, calculate the estimated time to travel to destination, and then surface once that time is reached.
        Latency is the amount of extra time it takes to calculate the while loop conditions and to execute it. This latency is added to END_TIME, because due to the extra time it takes to calculate conditions between telling the AUV to move, the estimated travel time to the destination will increase.
        '''
        travel_time = distance
        start_time = time.clock()
        latency =
        end_time = start_time + travel_time + latency
        while not self.surface() and bearing_check(bearing):
            if time.clock() < end_time:
                self.traverse()
                if self.sample() > threshold:
                    self.dense_traverse()
                continue
            else:
                #Once time is elapsed, surface and check GPS location
                self.velocity = 0
                return self.surface()

    #dense traverse function that is called when pollution is above a threshold
    def dense_traverse(self, start, end, distance, bearing, current = 1):
        '''
        dense traverse makes more loops within a smaller area. It accomplishes this by traveling 1/5 the width of sparse traverse for it's width portion, and minute steps for it's length.
        Calculate the cardinal directions that are orthogonal to BEARING on both sides.
        Determine the total width and length of the traversal area.
        '''
        #Calculate orthogonal cardinal directions from BEARING
        left_direction = bearing#rotate 90 deg ccw
        right_direction = bearing#rotate 90 deg cw

        #Orient vehicle in leftwards orthogonal cardinal direction
        self.bearing_check(left_direction)
        for i in range(distance/2):
            self.traverse(time)



    #handle a mavlink packet
    def mavlink_packet(self, m):


def init(mpstate):
    return AUVModule(mpstate)
