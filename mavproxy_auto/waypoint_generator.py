#!/usr/bin/env python

from pymavlink import mavwp as wp

'''Creates waypoints from lat/lng coordinates sent to it from a map API'''

'''listen on the port from the app'''

# append points to list
wp_list = []

# write points to file
with open('/home/pi/waypoints.txt', "a+") as f:
    f.write(''.join(wp_list))

# send points straight to mavproxy
for wps in wp_list:
    wp.add_latlonalt(wps.lat, wps.lng, 0)
