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
    #Determines whether to surface
    def surface(battery, bearing, time):
        if battery <= 0.2 or time >= 180:
            mav.#surface command, spin vertical motors
            return false
        else:
            return true

    #Check that the AUV is facing the correct cardinal direction
    def bearingCheck(currentBearing, intendedBearing):
        
