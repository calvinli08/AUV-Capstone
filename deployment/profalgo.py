#This is the Python implementation of the sparse/dense traversal algorithm outlined in
#Pompili et al. 2012
#It directs the AUV to travel in a sparse lawnmower pattern across the mission area.
#If the AUV detects a pollution concentration greater than a certain threshold during the sparse traversal
#it will automatically switch into a dense traversal mode, performing denser lawnmower sweeps in the direction of the
#pollution. Afterwards, it will return to sparse sweeping.
#Users can also enter Points of Interest to the AUV, which will be examined by the AUV using dense traverses
#after completing the whole sparse traversal. POI will be visited in the order given by a nearest neighbor algorithm.

from pymavlink import mavutil as mav
import os
from MAVProxy.modules import mavproxy_battery as bm
from MAVProxy.modules import mavproxy_DGPS as dgps

class AUV:
    #__init__
    def __init__(self):
        log = open('''path to log''',"r")
        self.battery = bm.percentage()
        self.bearing = log.readline()
        self.velocity = 0
        self.magX =
        self.magY =
        self.magZ =
        self.time = 0
        self.

    #updates all the navigational parameters. Essentially __init__.
    def update(self):
        log = open('''path to log''',"r")
        self.battery = bm.percentage()
        self.bearing = log.readline()
        self.velocity = log.readline()
        self.magX =
        self.magY =
        self.magZ =
        self.time =

    #Performs pre-dive information gathering and checks
    def predive(self):
        #Get the current GPS coordinate and the next waypoint
        self.GPSupdate(self)
        #Detect the river's present current
        #Is there a module for this?
        #Check if stats are fit for diving i.e. is there enough battery, mission area, etc
        self.update()
        #Perform arm_throttle checks and any QGroundControl checks
        qgc_checks = #boolean
        if self.battery >= 60.0 and qgc_checks:
            self.dive('''cardinal direction of next GPS waypoint''')
            return true
        elif 35.0 <= self.battery < 60.0 and qgc_checks:
            #If the distance is within coverable distance, dive
            if self.battery*time_multiplier - (distance / self.velocity) > '''enough time to surface''':
                self.dive('''cardinal direction of next GPS waypoint''')
                return true
            else:
                #Do nothing
                return false
        else:
            #Do nothing
            return false

    #Dives to a set depth, turns on the lights, and travels at a constant velocity to target
    def dive(self, depth, bearing, current):



    #Determines whether to surface
    def surface(self):
        #BATTERY is measured as a percentage
        #TIME is in minutes
        #VELOCITY is in meters/second
        self.update()
        if self.battery <= 35.0 or self.time >= 180 or self.velocity < :
            mav.#surface command, spin vertical motors
            self.predive()
            return false
        else:
            return true

    #Check that the AUV is facing the correct cardinal direction
    def bearingCheck(self, intendedBearing):
        self.update()
        #Calculate offset from intended bearing
        xOffset = self.magX - intendedBearing['magX']
        yOffset = self.magY - intendedBearing['magY']
        zOffset = self.magZ - intendedBearing['magZ']

        #


    #Sparse Traverse function
    #The core of the AUV class: directs the AUV to travel in a sparse lawnmower pattern around the mission area
    def sparse_traverse(self):
