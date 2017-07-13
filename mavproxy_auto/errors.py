#!/usr/bin/env python


class HardwareError(EnvironmentError):
    '''
    errno:
    0 - Irrecoverable Electrical Failure
    1 - Insufficient Battery
    2 - Low Battery
    3 - Lost Connection to Pixhawk
    4 - Motor Failure
    '''
    def __init__(self, args):
        self.errno = int(args[0])
        self.message = str(args[1])


class FileError(Exception):
    def __init__(self, message):
        self.message = message


class PWMError(EnvironmentError):
    def __init__(self, message):
        self.message = message
