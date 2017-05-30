#This is the Python implementation of the sparse/dense traversal algorithm outlined in
#Pompili et al. 2012
#It directs the AUV to travel in a sparse lawnmower pattern across the mission area.
#If the AUV detects a pollution concentration greater than a certain threshold during the sparse traversal
#it will automatically switch into a dense traversal mode, performing denser lawnmower sweeps in the direction of the
#pollution. Afterwards, it will return to sparse sweeping.
#Users can also enter Points of Interest to the AUV, which will be examined by the AUV using dense traverses
#after completing the whole sparse traversal. POI will be visited in the order given by a nearest neighbor algorithm.

import mavutils as mav
import os

class AUV:
    #__init__
    def __init__(self):
        log = open('''path to log''',"r")
        self.battery = log.readline()
        self.bearing = log.readline()
        self.velocity = log.readline()
        self.magX =
        self.magY =
        self.magZ =
        self.time =

    #updates all the navigational parameters. Essentially __init__.
    def update(self):
        log = open('''path to log''',"r")
        self.battery = log.readline()
        self.bearing = log.readline()
        self.velocity = log.readline()
        self.magX =
        self.magY =
        self.magZ =
        self.time =

    #Determines whether to surface
    def surface(self, battery, bearing, time, velocity):
        #BATTERY is measured as a percentage
        #TIME is in minutes
        #VELOCITY is in meters/second
        self.update(self)
        if self.battery <= 0.2 or self.time >= 180 or self.velocity < :
            mav.#surface command, spin vertical motors
            return false
        else:
            return true

    #Check that the AUV is facing the correct cardinal direction
    def bearingCheck(currentBearing, intendedBearing):
        
