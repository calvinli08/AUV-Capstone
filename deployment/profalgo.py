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
Waypoints will be fed into the AUV class upon __init__ from QGroundControl file where they're stored
'''

from pymavlink import mavutil as mav
import os, time
from MAVProxy.modules import mavproxy_battery as bm
from MAVProxy.modules import mavproxy_DGPS as dgps
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting

class AUVModule(mp_module.MPModule):
    #__init__
    def __init__(self):
        with open('''path to log''',"r") as log:
            self.battery = bm.BatteryModule.percentage()
            self.velocity = 0
            self.magX =
            self.magY =
            self.magZ =
            #update using the dataflash logs
            self.time = 0
            self.surfaced = true
            #GPS dependent values
            self.location = []
            self.waypoints = []
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
        #Get the current GPS coordinate and the next waypoint
        self.gps_update()
        #Detect the river's present current
        #Is there a module for this?
        #Check if stats are fit for diving i.e. is there enough battery, mission area, etc
        self.update()
        #Perform arm_throttle checks and any QGroundControl checks. Also, set a home waypoint.

        qgc_checks = #boolean
        if self.battery >= 60.0 and qgc_checks:
            #orient the AUV in the correct cardinal direction
            self.bearingCheck(self.intended_bearing)
            self.dive(1, self.intended_bearing, 1)
            return true
        elif 35.0 <= self.battery < 60.0 and qgc_checks:
            #If the distance is within coverable distance, dive
            if self.battery*time_multiplier - (distance / self.velocity) > '''enough time to surface''':
                self.dive('''cardinal direction of next GPS waypoint''')
                return true
            else:
                #Return to the home point
                self.home()
                return false
        else:
            #Return to the home point
            self.home()
            return false

    #Dives to a set depth, turns on the lights, and travels at a constant velocity to target
    def dive(self, depth, bearing, current):

        #call the mavproxy command to spin motors down till the required depth
        mav.
        self.surfaced = false
        self.sparse_traverse(self.location, self.waypoints.front(), , )

    #Determines whether to surface
    def surface(self):
        '''
        BATTERY is measured as a percentage
        TIME is in minutes
        VELOCITY is in meters/second
        '''
        self.update()
        if self.battery <= 35.0 or self.time >= 180 or self.velocity < :
            mav.#surface command, spin vertical motors
            self.surfaced = true
            self.predive()
            return false
        else:
            return true

    #Check that the AUV is facing the correct cardinal direction and correct it if necessary
    def bearing_check(self, intendedBearing):
        self.update()
        #Calculate offset from intended bearing
        xOffset = self.magX - intendedBearing['magX']
        yOffset = self.magY - intendedBearing['magY']
        zOffset = self.magZ - intendedBearing['magZ']

        #Correct the error
        mav.#change yaw
        mav.#change pitch
        mav.#change roll

    #traverse function
    #The core of the AUV class: directs the AUV to travel from it's current location to a waypoint
    def underwater_traverse(self, start, end, distance, bearing, current):
        '''
        Tell the AUV to move at a constant speed in the bearing direction while accounting for current.
        In order to determine when to surface, calculate the estimated time to travel to destination, and then surface once that time is reached.
        Latency is the amount of extra time it takes to calculate the while loop conditions and to execute it. This latency is added to END_TIME, because due to the extra time it takes to calculate conditions between telling the AUV to move, the estimated travel time to the destination will increase.
        '''
        travel_time = #distance in meters
        start_time = time.now()
        end_time = start_time + travel_time + latency
        while !self.surface() and bearing_check(bearing):
            if time.now() < end_time:
                mav.#move forward at 1 m/s
                mav.#spin thrusters in opposite direction of current
                if pollution > threshold:
                    self.dense_traverse()
                continue
            else:
                #Once time is elapsed, surface and check GPS location
                self.velocity = 0
                return self.surface()

    #dense traverse function that is called when pollution is above a threshold
    def dense_traverse(self, start, end, distance, bearing):
        '''
        dense traverse makes more loops within a smaller area. It accomplishes this by traveling 1/5 the width of sparse traverse for it's width portion, and minute steps for it's length.
        '''
        travel_time =

    #handle a mavlink packet
    def mavlink_packet(self, m):


def init(mpstate):
    return AUVModule(mpstate)
