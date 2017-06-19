#!/usr/bin/python 

# uses the curses library to make a terminal screen that allows
# the user to communicate with Atlas Scientific boards

import serial # required for communication with boards
import RPi.GPIO as GPIO
from time import strftime, sleep # used for timestamps, delays

class SerialReader():
	def __init__(self):
		  self.usbport = '/dev/ttyAMA0'
    	self.ser = serial.Serial(self.usbport, 9600, timeout = 0)
    	self.line = ""

    	GPIO.setmode(GPIO.BCM)
    	self.S0_pin = 18
    	self.S1_pin = 23
    	self.S2_pin = 24

    	GPIO.setup(self.S0_pin, GPIO.OUT) # S0 
    	GPIO.setup(self.S1_pin, GPIO.OUT) # S1
    	GPIO.setup(self.S2_pin, GPIO.OUT) # S0 

	def set_channel(self,channel):
	    # sets the multiplexer to the specified channel
	    # channel name is a string, not an int
	    if channel == '0':
	    	GPIO.output(self.S0_pin, False)
	    	GPIO.output(self.S1_pin, False)
	    	GPIO.output(self.S2_pin, False)

	    elif channel == '1':
	        GPIO.output(self.S0_pin, True)
	        GPIO.output(self.S1_pin, False)
	        GPIO.output(self.S2_pin, False)
	            
	    elif channel == '2':
	        GPIO.output(self.S0_pin, False)
	        GPIO.output(self.S1_pin, True)
	        GPIO.output(self.S2_pin, False)

	    elif channel == '3':
	        GPIO.output(self.S0_pin, True)
	        GPIO.output(self.S1_pin, True)
	        GPIO.output(self.S2_pin, False)

	    elif channel == '4':
	        GPIO.output(self.S0_pin, False)
	        GPIO.output(self.S1_pin, False)
	        GPIO.output(self.S2_pin, True)

	    elif channel == '5':
	        GPIO.output(self.S0_pin, True)
	        GPIO.output(self.S1_pin, False)
	        GPIO.output(self.S2_pin, True)

	    elif channel == '6':
	        GPIO.output(self.S0_pin, False)
	        GPIO.output(self.S1_pin, True)
	        GPIO.output(self.S2_pin, True)

	    elif channel == '7':
	        GPIO.output(self.S0_pin, True)
	        GPIO.output(self.S1_pin, True)
	        GPIO.output(self.S2_pin, True)
	    sleep(1)
		  self.ser.flushInput() # clear the data received on the previous channel
		  self.line = ""
	
	def read(self,channel):
    	self.set_channel(channel)
      data = self.ser.read() # get serial data
      
      while (data != "\r"): # if its not terminated by a newline
        data = self.ser.read() # get serial data
        self.line  = self.line + data # if the line isn't complete, add the new characters to it
      
      result = self.line
      self.line = "" # clean up line
      return result 
            
