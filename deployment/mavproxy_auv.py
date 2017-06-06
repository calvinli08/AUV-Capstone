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

from pymavlink import mavutil as mav
from collections import deque
import os, time, math
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
            #update using the dataflash logs
            self.velocity = 0
            self.time = time.clock()
            self.surfaced = True
            #GPS dependent values
            self.location = mav.mavfile.location()
            self.home = [self.location.lat, self.location.lng]
            self.waypoints = deque(wp.WPModule.load_waypoints_dive('/home/pi/waypoints.txt'))
            wp = self.waypoints.popleft().rsplit()
            self.next_wp = [float(wp[3]), float(wp[2])] #lng,lat
            self.heading = self.location.heading #degrees relative to Magnetic North
            self.distance = self._haversine(self.location.lng, self.location.lat, self.next_wp[0], self.next_wp[1]) #meters
            self.intended_heading = self._intended_heading( self.heading, self.location.lng, self.location.lat, self.next_wp[0], self.next_wp[1] )#degrees relative to Magnetic North
            #need a current sensor for this. Let's try to find average current info online and hardcode it for now.
            self.current = #m/s
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
            mav.mavfile.set_mode_manual()

    #calculate the distance between two latlng points using the haversine formula
    #source: https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
    def _haversine(self, lon1, lat1, lon2, lat2):
        """
        Calculate the great circle distance between two points
        on the earth (specified in decimal degrees)
        """
        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(math.radians(), [lon1, lat1, lon2, lat2])

        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371 # Radius of earth in kilometers. Use 3956 for miles
        return c * r * 1000

    #Calculate intended heading
    #This is assuming that heading is measured relative to Magnetic North
    def _intended_heading(self, present_heading, lng1, lat1, lng2, lat2):
        #determine quadrant of next waypoint
        #calculate atan2
        #rotate answer according to waypoints original quadrant

        #corner of the right triangle formed using location and the next waypoint as it's vertices
        rt_corner = [lng2, lat1]

        #The list index in quandrant that is TRUE is the quadrant the next waypoint is in
        quadrant = [lng2 > lng1 && lat2 > lat1, lng2 < lng1 && lat2 > lat1, lng2 < lng1 && lat2 < lat1, lng2 > lng1 && lat2 < lat1]

        #Uncorrected heading calculated from atan2 which will be in first quadrant. Measured relative to East
        uncorrected_heading = math.degrees(math.atan2( self._haversine(rt_corner[0], rt_corner[1], lng2, lat2) / self._haversine(lng1, lat1, rt_corner[0], rt_corner[1]) )) #degrees

        if quadrant[0]:
            return 90 - uncorrected_heading
        elif quadrant[1]:
            return (90 - uncorrected_heading) + (2 * uncorrected_heading + 180)
        elif quadrant[2]:
            return 90 - uncorrected_heading + 180
        else:
            return (90 - uncorrected_heading) + (2 * uncorrected_heading)

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
            mav.mavfile.reboot_autopilot()
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
        with open('''path to log''',"r") as log:
            self.battery = bm.BatteryModule.percentage()
            self.heading =
            self.velocity = log.readline()
            self.time = time.clock()
            self.current =

    #Update the AUV's current GPS location, and get the cardinal direction of the next waypoint, and distance to it.
    def gps_update(self, next_wp):
        #No point in checking it underwater where there is no GPS fix. That might also feed corrupted data to the module, so avoid calling this underwater.
        if self.surfaced == True:
            self.location = mav.mavfile.location()
            self.distance = self._haversine(self.location.lng, self.location.lat, next_wp[0], next_wp[1])
            self.heading = self.location.heading
            self.intended_heading = self._intended_heading(self.heading, self.location.lng, self.location.lat, next_wp[0], next_wp[1])
        else:
            return

    #Maneuvers the AUV while on the surface back towards the home point for maintenance, etc
    def home(self):
        #Traverse to home waypoint
        while self.location != self.home:
            self.update()
            self.gps_update(self.home)
            self.heading_check(self.intended_heading)
            mav.#spin forward propellers

    #Performs pre-dive information gathering and checks
    def predive(self):
        #Get the current GPS coordinate
        self.gps_update(self.next_wp)

        #Set the next waypoint if the last waypoint was reached
        #If no more waypoints, go home
        if not self.waypoints:
           self.home()
        elif self.location == self.next_wp:
           self.next_wp = self.waypoints.popleft()

        #Detect the river's present current
        #Is there a module for this?
        #Check if stats are fit for diving i.e. is there enough battery, mission area, etc
        self.gps_update(self.next_wp)
        self.update()

        #Perform arm_throttle checks and any QGroundControl checks.
        qgc_checks = #boolean

        #Calculate distance between dive_point and next_wp
        distance = self._haversine(self.location.lng, self.location.lat, self.next_wp[0], self.next_wp[1])
        #Orient the vehicle in the correct cardinal direction
        self.heading_check(self.intended_heading)

        if self.battery >= 60.0 and qgc_checks:
            #Since there is enough battery, dive
            self.dive(self.location, self.next_wp, distance, 1, self.intended_heading, 1)
            return True
        elif 35.0 <= self.battery < 60.0 and qgc_checks and (self.battery*time_multiplier - (distance / self.velocity)) > 2:
            #If the distance is within coverable distance, dive
            #Units for above boolean condition is seconds
            #Dive
            self.dive(self.location, self.next_wp, distance, 1, self.intended_heading, 1)
            return True
        else:
            #Return to the home point
            self.home()
            return False

    #Dives to a set depth, turns on the lights, and travels at a constant velocity to target
    def dive(self, dive_point, destination, distance, depth = 1, heading, current = 1):
        #call the mavproxy command to spin motors down till the required depth
        for i in range(0, depth, 0.5):
            mav.
        self.surfaced = False
        self.underwater_traverse(dive_point, destination, distance, heading, current)

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
            mav.#surface command, spin vertical motors
            self.surfaced = True
            self.predive()
            return True
        else:
            return False

    #Check that the AUV is facing the correct cardinal direction and correct it if necessary
    def heading_check(self, intended_heading):
        self.update()
        #Calculate offset from intended heading
        offset = self.heading - intended_heading
        if math.abs(offset) > 5:
            #Correct the error
            if offset > 0:
                mav.#change yaw ccw
            else:
                mav.#change yaw cw
        else:
            return

    #Sample pollution with the sensors
    def sample(self):
        #read from serial in

        return #pollution level as a float

    #traverse
    def traverse(self, time):
        while mav.mavfile.motors_armed():
        mav.#move forward at 1 m/s
        mav.#spin thrusters in opposite direction of current
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
        while self.surface() is False and self.heading_check(heading):
            if end_time - time.clock() >= 0:
                if self.traverse() > threshold:
                    remaining_distance = end_time - self.dense_traverse(,, '''Calculate distance based on ''', heading, 1)
                    end_time = time.clock() + remaining_distance
                else:
                #Once time is elapsed, surface and check GPS location
                self.velocity = 0
                return self.surface()

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
        self.heading_check(left_direction)
        for i in range(distance/2):
            self.traverse(time)


        return time.clock() - start_time

    #handle a mavlink packet
    def mavlink_packet(self, m):


def init(mpstate):
    return AUVModule(mpstate)
