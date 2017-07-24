#!/usr/bin/env python

import os

'''Creates waypoints from lat/lng coordinates sent to it from a map API'''

'''listen on the port from the app - implement when have time'''

index = 0
wp = []

if os.path.getsize('/home/pi/wps.txt') is 0:
    with open('/home/pi/wps.txt', "w") as f:
        f.write("QGC WPL 110\n")

with open('/home/pi/waypoints.txt', 'a+') as f:
    _coor = f.readlines()

for wayp in _coor:
    latlng = wayp.strip().split(',')

    latlng[0] = float(latlng[0])
    latlng[1] = float(latlng[1])

    print latlng

    if index is 0:
        wp.append("%s\t1\t0\t16\t0.149999999999999994\t0\t0\t0\t%s\t%s\t0\t1\n" % (index, latlng[1], latlng[0]))
    else:
        wp.append("%s\t0\t0\t16\t0.149999999999999994\t0\t0\t0\t%s\t%s\t0\t1\n" % (index, latlng[1], latlng[0]))

    # write points to file
    with open('/home/pi/wps.txt', "a+") as d:
        d.write(wp[index])

    index += 1
