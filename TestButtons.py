
import os
import subprocess
from PIL import Image
import time
import datetime
import picamera as picam
from collections import defaultdict, deque, OrderedDict
import datetime
from PIL import Image
import math
import  RPi.GPIO as GPIO            # used for GPIO switches in state 1 
import smbus as smbus                       # used for peripheral calls to both the IMU and haptic driver
import subprocess
import time

''' GPIO setup'''
# Using the pin layout on the board, where 1 is top left
GPIO.setmode(GPIO.BOARD)

# Setup buttons for selecting destination as user input
select_next = 12    # GPIO_GEN1, GPIO18, or pin number 12 on the board
select_prev = 16    # GPIO_GEN4, GPIO23, or pin number 16 on the board
select_current = 18 # GPIO_GEN5, GPIO25, or pin number 18 on the board

# setup GPIO pins as low

# setup GPIO buttons as input with pull up
GPIO.setup(select_next, GPIO.IN)
GPIO.setup(select_prev, GPIO.IN)
GPIO.setup(select_current, GPIO.IN)

class Input():
    
    def __init__(self):
        self.max_count = 100


    def debounce(self, gpio_pin):
        counter = 0
        button_state = False
        
        for count_on in range(100):
            if GPIO.input(gpio_pin):
                counter += 1

        if counter >= self.max_count and GPIO.input(gpio_pin):
            button_state = True

        return button_state
    
    def deb_mult(self, pins):
        counter = 0
        button_state = False
        button = 0
        
        for count_on in range(100):
            if len(pins) > 0:
                for length in range(len(pins)):
                    if GPIO.input(pins[length]):
                        button = pins[length]
                        counter += 1
          #  else:
                # oops! what to do?
  
                    

        if counter >= self.max_count and GPIO.input(pins[button]):
            button_state = True

        return button_state

button_input = Input()
while True:
    if GPIO.input(select_next) == True:
        if button_input.debounce(select_next) == True:
            print(" select_prev")
    if GPIO.input(select_prev) == True:
        if button_input.debounce(select_prev) == True:
            print("Previous Selected")
    if GPIO.input(select_current) == True:
        if button_input.debounce(select_current) == True:
            print("Current Selected")
