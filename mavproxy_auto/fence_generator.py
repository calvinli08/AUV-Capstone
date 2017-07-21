#!/usr/bin/env python

'''Creates geofence from lat/lng coordinates sent to it from a map API'''

'''listen on the port from the app - implement when have time'''

index = 0

while True:

    coor = raw_input('Enter fence point: ')

    _coor = coor.split()

    wp = "%s\t1\t0\t16\t0.149999999999999994\t0\t0\t0\t0\t%s\t%s\t0\t1" % (index, _coor[longitude], _coor[latitude])

    index += 1

    # write points to file
    with open('/home/pi/fence.txt', "a+") as f:
        f.write(wp)
