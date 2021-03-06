# Imports

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

# setup GPIO buttons as input
GPIO.setup(select_next, GPIO.IN)
GPIO.setup(select_prev, GPIO.IN)
GPIO.setup(select_current, GPIO.IN)

''' End GPIO setup'''

# Class used for processing input, such as debounce
class Input():
    
    def __init__(self):
        self.max_count = 50


    def debounce(self, gpio_pin):
        counter = 0
        button_state = False
        
        for count_on in range(100):
            if GPIO.input(gpio_pin):
                counter += 1

        if counter >= self.max_count and GPIO.input(gpio_pin):
            button_state = True

        return button_state
    
    def deb_mult(slef, pins):
        counter = 0
        button_state = False
        
        for count_on in range(100):
            for length in range(len(pins)):
                if GPIO.input(pins(length)):
                    counter += 1

        if counter >= self.max_count and GPIO.input(gpio_pin):
            button_state = True

        return button_state



''' MPU-9250 Definitions '''
# coding: utf-8
# Greg Briggs
#  MPU_9250 code adapted from FaBo9Axis_MPU9250 library for the FaBo 9AXIS I2C Brick 
#  and translatiion from Arduino to Python from kriswiner/MPU9250 on GitHub. 
#  Functions written by Greg Briggs include changeOfBasis and readIMU functions.
#  http://fabo.io/202.html
#  https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
#  https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
#  
#  Adapted source code was 
#  released under APACHE LICENSE, VERSION 2.0
#
#  http://www.apache.org/licenses/
#
#  FaBo <info@fabo.io>



## MPU9250 Default I2C slave address
SLAVE_ADDRESS        = 0x68
## AK8963 I2C slave address
AK8963_SLAVE_ADDRESS = 0x0C
## Device id
DEVICE_ID            = 0x71

''' MPU-9250 Register Addresses '''
## sample rate driver
SELF_TEST_X_GYRO = 0x0
SELF_TEST_Y_GYRO = 0x1
SELF_TEST_Z_GYRO = 0x2
SELF_TEST_X_ACCEL = 0xD
SELF_TEST_Y_ACCEL = 0xE
SELF_TEST_Z_ACCEL = 0xF
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
LP_ACCEL_ODR   = 0x1E
WOM_THR        = 0x1F
FIFO_EN        = 0x23
I2C_MST_CTRL   = 0x24
I2C_MST_STATUS = 0x36
INT_PIN_CFG    = 0x37
INT_ENABLE     = 0x38
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43

XG_OFFSET_H    = 0x13
XG_OFFSET_L    = 0x14
YG_OFFSET_H    = 0x15
YG_OFFSET_L    = 0x16
ZG_OFFSET_H    = 0x17
ZG_OFFSET_L    = 0x18
ACCEL_XOUT_H   = 0x3B
ACCEL_XOUT_L   = 0x3C
ACCEL_YOUT_H   = 0x3D
ACCEL_YOUT_L   = 0x3E
ACCEL_ZOUT_H   = 0x3F
GYRO_XOUT_H    = 0x43
GYRO_XOUT_H    = 0x44
GYRO_XOUT_H    = 0x45
GYRO_XOUT_H    = 0x46
GYRO_XOUT_H    = 0x47
GYRO_XOUT_H    = 0x48
ACCEL_ZOUT_L   = 0x40
XA_OFFSET_H    = 0x77
XA_OFFSET_L    = 0x78
YA_OFFSET_H    = 0x7A
YA_OFFSET_L    = 0x7B
ZA_OFFSET_H    = 0x7D
ZA_OFFSET_L    = 0x7E

I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET  = 0x68
MOT_DETECT_CTRL    = 0x69
USER_CTRL          = 0x6A
PWR_MGMT_1         = 0x6B
PWR_MGMT_2         = 0x6C
FIFO_COUNTH        = 0x72
FIFO_R_W           = 0x74
WHO_AM_I           = 0x75

## Gyro Full Scale Select 250dps
GFS_250  = 0x00
## Gyro Full Scale Select 500dps
GFS_500  = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000 = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000 = 0x03
## Accel Full Scale Select 2G
AFS_2G   = 0x00
## Accel Full Scale Select 4G
AFS_4G   = 0x01
## Accel Full Scale Select 8G
AFS_8G   = 0x02
## Accel Full Scale Select 16G
AFS_16G  = 0x03

# AK8963 Register Addresses
AK8963_ST1        = 0x02
AK8963_MAGNET_OUT = 0x03
AK8963_CNTL1      = 0x0A
AK8963_CNTL2      = 0x0B
AK8963_ASAX       = 0x10

# CNTL1 Mode select
## Power down mode
AK8963_MODE_DOWN   = 0x00
## One shot data output
AK8963_MODE_ONE    = 0x01

## Continous data output 8Hz
AK8963_MODE_C8HZ   = 0x02
## Continous data output 100Hz
AK8963_MODE_C100HZ = 0x06

# Magneto Scale Select
## 14bit output
AK8963_BIT_14 = 0x00
## 16bit output
AK8963_BIT_16 = 0x01

# Pin definitions
intPin = 12 # These can be changed, 2 and 3 are the Arduinos ext int pins

accelCount = [0.0, 0.0, 0.0]# Stores the 16-bit signed accelerometer sensor output
gyroCount = [0.0, 0.0, 0.0] # Stores the 16-bit signed gyro sensor output
magCount = [0.0, 0.0, 0.0]# Stores the 16-bit signed magnetometer sensor output
magCalibration = [0.0, 0.0, 0.0] 
magbias = [0.0, 0.0, 0.0] # Factory mag calibration and mag bias
gyroBias = [0.0, 0.0, 0.0] 
accelBias = [0.0, 0.0, 0.0] # Bias corrections for gyro and accelerometer
ax, ay, az, gx, gy, gz, mx, my, mz = (0, 0, 0, 0, 0, 0, 0, 0, 0) # variables to hold latest sensor data values 
tempCount = 0 # Stores the real internal chip temperature in degrees Celsius
temperature = 0.0
SelfTest = [0.0,0.0,0.0,0.0,0.0,0.0]

delt_t = 0 # used to control display output rate
count = 0 # used to control display output rate

# parameters for 6 DoF sensor fusion calculations
GyroMeasError = math.pi * (60.0 / 180.0)     # gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
beta = math.sqrt(3.0 / 4.0) * GyroMeasError  # compute beta
GyroMeasDrift = math.pi * (1.0 / 180.0) #gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
zeta = math.sqrt(3.0 / 4.0) * GyroMeasDrift  # compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
# Check values for Kp and Ki, as the documentation commented out these variables with the values assigned below
Kp = 2.0 * 10.0     #define Kp 2.0 * 5.0 # these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
Ki = 0.0            #define Ki 0.0

pitch, yaw, roll = (0.0, 0.0, 0.0)  # Instantiation for quaterion-calculated ptch yaw roll
deltat = 0.0                      # integration interval for both filter schemes
lastUpdate = 0  			# used to calculate integration interval 
firstUpdate = 0  			# used to calculate integration interval 
Now = 0                            	# used to calculate integration interval                             
q = [1.0, 0.0, 0.0, 0.0]     	# vector to hold quaternion
eInt = [0.0, 0.0, 0.0]             	# vector to hold integral error for Mahony method

## smbus
bus = smbus.SMBus(1)
''' End of MPU-9250 Register Addresses '''

''' Beginning of Haptic Motor Driver Addresses and Code'''
bus = smbus.SMBus(1) # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

DRV2605_ADDR = 0x5A

# Feedback Effects
BUZZ_100 = 47

DRV2605_REG_STATUS = 0x00
DRV2605_REG_MODE = 0x01
DRV2605_MODE_INTTRIG = 0x00
DRV2605_MODE_EXTTRIGEDGE = 0x01
DRV2605_MODE_EXTTRIGLVL = 0x02
DRV2605_MODE_PWMANALOG = 0x03
DRV2605_MODE_AUDIOVIBE = 0x04
DRV2605_MODE_REALTIME = 0x05
DRV2605_MODE_DIAGNOS = 0x06
DRV2605_MODE_AUTOCAL = 0x07


DRV2605_REG_RTPIN = 0x02
DRV2605_REG_LIBRARY = 0x03
DRV2605_REG_WAVESEQ1 = 0x04
DRV2605_REG_WAVESEQ2 = 0x05
DRV2605_REG_WAVESEQ3 = 0x06
DRV2605_REG_WAVESEQ4 = 0x07
DRV2605_REG_WAVESEQ5 = 0x08
DRV2605_REG_WAVESEQ6 = 0x09
DRV2605_REG_WAVESEQ7 = 0x0A
DRV2605_REG_WAVESEQ8 = 0x0B

DRV2605_REG_GO = 0x0C
DRV2605_REG_OVERDRIVE = 0x0D
DRV2605_REG_SUSTAINPOS = 0x0E
DRV2605_REG_SUSTAINNEG = 0x0F
DRV2605_REG_BREAK = 0x10
DRV2605_REG_AUDIOCTRL = 0x11
DRV2605_REG_AUDIOLVL = 0x12
DRV2605_REG_AUDIOMAX = 0x13
DRV2605_REG_RATEDV = 0x16
DRV2605_REG_CLAMPV = 0x17
DRV2605_REG_AUTOCALCOMP = 0x18
DRV2605_REG_AUTOCALEMP = 0x19
DRV2605_REG_FEEDBACK = 0x1A
DRV2605_REG_CONTROL1 = 0x1B
DRV2605_REG_CONTROL2 = 0x1C
DRV2605_REG_CONTROL3 = 0x1D
DRV2605_REG_CONTROL4 = 0x1E
DRV2605_REG_VBAT = 0x21
DRV2605_REG_LRARESON = 0x22
#
class Adafruit_DRV2605():        
    
    def __init__(self):
        self.effect = 0
    
    def begin():
            
        # save the device address 
        id = read_byte_data(DRV2605_ADDR, DRV2605_REG_STATUS)
        #Serial.print("Status 0x") Serial.println(id, HEX)
        
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_MODE, 0x00) # out of standby
        
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_RTPIN, 0x00) # no real-time-playback
        
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_WAVESEQ1, 1) # strong click
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_WAVESEQ2, 0)
        
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_OVERDRIVE, 0) # no overdrive
        
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_SUSTAINPOS, 0)
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_SUSTAINNEG, 0)
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_BREAK, 0)
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_AUDIOMAX, 0x64)
        
        # ERM open loop
        
        # turn off N_ERM_LRA
        bus.write_byte_data(DRV2605_ADDR,DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) & 0x7F)
        # turn on ERM_OPEN_LOOP
        bus.write_byte_data(DRV2605_ADDR,DRV2605_REG_CONTROL3, readRegister8(DRV2605_REG_CONTROL3) | 0x20)

        return true


    def setWaveform(slot, w):
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_WAVESEQ1+slot, w)


    def selectLibrary(lib):
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_LIBRARY, lib)


    def go():
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_GO, 1)


    def stop():
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_GO, 0)


    def setMode(mode):
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_MODE, mode)


    def setRealtimeValue(rtp):
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_RTPIN, rtp)


    def useERM ():
            bus.write_byte_data(DRV2605_ADDR,DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) & 0x7F)


    def useLRA():
            bus.write_byte_data(DRV2605_ADDR,DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) | 0x80)
            
    def playEffect(self, effect):
        for packet in range (len(effect)):
            # set the effect to play
            Adafruit_DRV2605.setWaveform(packet, effect[packet])# play effect
        
        # play the effect!
        Adafruit_DRV2605.go()
    
#
''' End of Haptic Motor Driver Addresses '''

#qrcodeGUI.py

class SET():
    PV_SIZE=(320,240)
    NORM_SIZE=(2592,1944)
    NO_RESIZE=(0,0)
    PREVIEW_FILE="PREVIEW.gif"
    TEMP_FILE="PREVIEW.ppm"
    QR_SIZE=(640,480)
    READ_QR="zbarimg "

class cameraGUI():

    def __init__(self):
        
        self.scan=False
        self.resultQR =""

    def run_p(cmd):

        proc=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE)
        result=""
        for line in proc.stdout:
            result=str(line,"utf-8")

        return result

    def camCapture(filename,size=SET.NORM_SIZE):
        with picam.PiCamera() as camera:
            camera.resolution = size
            camera.capture(filename)

    # Use the timestamp 
    def timestamp():
        ts=time.time()
        tstring=datetime.datetime.fromtimestamp(ts)
        return tstring.strftime("%Y%m%d_%H%M%S")

    def qrGet(self):

        if (self.scan==True):

            self.scan=False

        else:

            self.scan=True

            self.qrScanner()

    def qrScanner(self):
	
        # ensures the result is found by setting found to false
        found=False
		
        while self.scan == True:
            cameraGUI.camCapture(SET.PREVIEW_FILE,SET.QR_SIZE)

        #check for QR code in image
        qrcode=cameraGUI.run_p(SET.READ_QR+SET.PREVIEW_FILE)

        if len(qrcode)>0:
            # delete this line after removing the line that adds the string "QR-Code"
            qrcode=qrcode
            qrcode=qrcode.strip("QR-Code:").strip('\n')
            self.resultQR.set(qrcode)
            self.scan=False

            found=True
            
        else:
            self.resultQR = ""
			return self.resultQR
		
        if self.qrRead.get() == 1:
            cameraGUI.run("sudo flite -t '"+qrcode+"'")

        if self.qrStream.get() == 1:
            cameraGUI.run("omxplayer '"+qrcode+"'")

        if self.qrRead.get() == 0 and self.qrStream.get() == 0:
            TK.messagebox.showinfo("QR Code",self.resultQR.get())
            self.resultQR.set("")

        if found:

            self.qrAction(qrcode)

        self.update()

    def qrAction(self,qrcode):
        self.resultQR = ""
        self.resultQR.set("")

        if found:
            self.resultQR.set("")

        if found:

            self.qrAction(qrcode)

        self.update()

    def qrAction(self,qrcode):
		self.qrAction(qrcode)
        self.update()

    def msg(self,text):
        self.filename.set(text)
        self.update()

    def normal(self):
        name=cameraGUI.timestamp()+".jpg"
        cameraGUI.camCapture(name,SET.NORM_SIZE)


#End

class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.rooms = OrderedDict()
        self.edges = defaultdict(list)
        self.Xdistances = {}    # fast initialization of an empty dict
        self.Ydistances = {}    # fast initialization of an empty dict
        self.vertices_2D = {}   # empty tuple

    # must either be an in or a string, a unique identifier for a node that is iterable in a for loop
    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, Xdistance, Ydistance):
        self.edges[from_node].append(to_node)       # appends to the entry from_node of dictionary "edges"
        self.edges[to_node].append(from_node)
        self.Xdistances[(from_node, to_node)] = Xdistance   # Define horizontal change in the edge
        self.Ydistances[(from_node, to_node)] = Ydistance   # Define vertical change in the edge
        self.vertices_2D[(from_node, to_node)] = [Xdistance,Ydistance]  # define vertices dictionary for 2 dimentions


    def get_dest(self, destination):
        for name in self.rooms.keys():       
            if destination in self.rooms[name]: 
                self.end = name
        return self.end

    def get_room(self, node):
        first_room = 0
        # scan dictionary keys 
        for key in self.rooms.keys():
            # for the passed node and
            if(key == node):
                # return first room number 
                first_room = self.rooms[key][0]
                # if value is found, immediately return to save clock cycles
                return first_room

        return first_room
            
        
    # when selecting user input, get the next room and node in the nodes dictionary
    # returns room number
    def next_room(self, key, current_room):
        # as long as rooms are ints, this local variable will work
        last_value = 0
        # Used to denote when the last entry in the dictionary is the current_room, 
        last_entry = False
        # local variable used for checks if the next key in ordered dict is necessary
        next_entry = False

        # get last dictionary value in dictionary
        for node, room in self.rooms.items():
            if len(room) > 1:
                for index in range(len(room)):
                    last_value = room[index]
            else:
                last_value = room[0]

        if last_value == current_room:
            last_entry = True
        
        # get nodes dictionary and find the current room
        for node, room in self.rooms.items():
            # if the last entry is the current_room, return the first value in the dictionary
            if last_entry == True:
                return room[0]
            
            # the next entry's first element in the dictionary must be returned
            if next_entry == True:
                return room[0]
                
            # found the correct key 
            elif key == node:
                # loop through set at dictionary entries that have another value
                if len(room) > 1:
                    for index in range(len(room)):
                        # if the value exists, return the next value in the array
                        if current_room is room[index] and index <= len(room) - 2:
                            # next room is the next room in the set at dictionary entry node
                            return room[index + 1]
                        
                # if return isn't used, allow for loop to iterate to next key:value pair
                next_entry = True

    # when selecting user input, get the prev room and node in the nodes dictionary
    # returns room number
    def prev_room(self, key, current_room):
        # store first node for looping
        first_node = ""
        
        # local variable used for checks if the next key in ordered dict is necessary
        prev_entry = False

        # first time iterating through the dictionary
        first_entry = False

        # only true when the first entry of the first key of the dictionary is the current_room
        go_last = False
        
        # get nodes dictionary and find the current room
        for node, room in self.rooms.items():

            if first_entry == False:
                # save the first entry in case we are at the first entry
                first_node = node
                # used to skip over this block
                first_entry = True
                if (first_node == key and current_room == room[0]):
                    # key is the first entry of the dictionary
                    go_last = True

                      
            # found the correct key and
            # room is not the first entry in the key in the dictionary
            if (key == node and go_last == False):
                # loop through set at dictionary entries that have another value
                if len(room) > 1:
                    for index in range(len(room)):
                        # if the value exists, return the prev value in the array,
                        # so long as that index value is greater than 0
                        if current_room is room[index] and index > 0:
                            # prev room is the next room in the set at dictionary entry node
                            return room[index - 1]
                        
                # if return isn't used, allow for loop to iterate to next key:value pair
                prev_entry = True
                
            # if node has not equaled key, save this node
            if prev_entry == False:
                prev_node = node
            # otherwise, node has equaled key, so return the last value of the previous key
            elif prev_entry == True and self.rooms[prev_node][len(self.rooms[prev_node])-1]:
                # return the last entry in the previous list at the key prev_node
                return self.rooms[prev_node][len(self.rooms[prev_node])-1]

        # outside while loop, if nothing else returned, pass last value of last node
        return self.rooms[prev_node][len(self.rooms[prev_node])-1]        

def SNIPE(graph, initial):
    visited = {initial: 0}
    path = {}

    nodes = set(graph.nodes)

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            try:
                # Start with current_weight = 0 for first pass
                # and add the distance information for the edge
                x_weight = current_weight + graph.Xdistances[(min_node, edge)]
                y_weight = current_weight + graph.Ydistances[(min_node, edge)]
                weight = math.sqrt(x_weight**2 + y_weight**2)
            except:
                continue
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path



# shortest_path
# type graph, char, char
# returns distance and path in single dict
def shortest_path(graph, origin, destination):
    # 
    visited, paths = SNIPE(graph, origin)   #
    full_path = deque()                     # empty deque, or a structure with streamlined append functions
    _destination = paths[destination]       # assign the destination to the path

    # reconstructs the path 
    while _destination != origin:
        # 
        full_path.appendleft(_destination)
        _destination = paths[_destination]

    # _destination == origin, so used saved passed variable destination. 
    full_path.appendleft(origin)
    full_path.append(destination)

    # returns dict with one entry, a magnitde of destination and a list of path nodes
    # note that destination is in centimeters or 1/100'
    return int(visited[destination]*100), list(full_path)

class Feedback():   
    
    def __init__(self):
        self.origin = {}    # where we start
        self.graph = {}         # 
        self.direction = {}     # empty dictionary
        self.path = set()  # get the path letters (it should be a list)
        self.poi_start = ""     # values that hold the nodes in the list value of path 
        self.poi_next = ""      # values that hold the nodes in the list value of path 
        self.poi_last = ""      # values that hold the nodes in the list value of path 
        self.effect = []        # stores the type of signal for the haptic driver
        self.prev_path = []   # simple value holding the value of the last path
        self.displacement = []  # used to store the (delta x, delta y) between an edge
        self.left = "left"
        self.right = "right"
        self.turn_around = "turn around"
        self.arrived = "arrived"
        self.choose_dest = "choose destination"
        self.room = 0
        self.forward = "forward"
        self.turn = ""
        arrived = "flite -t 'You have arrived at room " + str(self.room) + ".'"
        self.first_call = False
        self.room = 0     # used for giving the name of the room number (may not be required)
        self.turn_left_tts = 'flite -t "Turn left." '
        self.turn_right_tts = 'flite -t "Turn right." '
        self.turn_around_tts = 'flite -t "Please turn around." '
        self.select_room_tts = 'flite -t "Select your destination." '
        self.arrived_tts = "flite -t 'You have arrived at room " + str(self.room) + ".'"
        self.current_dest_tts = "flite -t 'Your current destination is room " + str(self.room) + ".'"
        self.get_confirm_dest_tts = "flite -t 'The selected destination is " + str(self.room) + ". Please confirm.'"
        self.confirm_dest_tts = "flite -t 'You have selected room " + str(self.room) + ". Scanning for current position.'"
        self.start_node = "flite -t 'You are currently standing by room " + str(self.room) + ". Plotting course.'"
        self.room_change_tts = "flite -t 'Room " + str(self.room) + ".'"
        self.location_found_tts = 'flite -t "Location found." '
        self.at_destination_tts = 'flite -t "You are standing by your desired destination." '
        self.forward_tts = 'flite -t "You are heading in the correct direction." '
        self.change_dest_detect_tts = 'flite -t "Button push detected. Would you like to change your destination?" '
        self.confrim_change_dest_tts = 'flite -t "Button push detected." '


    def update_room(self, room):
        self.arrived_tts = "flite -t 'You have arrived at room " + str(self.room) + ".'"
        self.current_dest_tts = "flite -t 'Your current destination is room " + str(self.room) + ".'"
        self.get_confirm_dest_tts = "flite -t 'The selected destination is " + str(self.room) + ". Please confirm.'"
        self.confirm_dest_tts = "flite -t 'You have selected room " + str(self.room) + ". Scanning for current position.'"
        self.start_node = "flite -t 'You are currently standing by room " + str(self.room) + ". Plotting course.'"
        self.room_change_tts = "flite -t 'Room " + str(self.room) + ".'"
    
        
    # direct(self)
    # compares two lists
    # returns direction in for of string
    def direct(self):
        # initialize variables each time.
        # if they were overwritten outside the function call to turn, it will not alter the behavior
        self.displacement = [0,0]
        self.poi_start = self.new_path[0]
        dispx = 0
        dispy = 0
        path_length = len(self.new_path)
            
        if path_length >= 2:
            self.poi_next = self.new_path[1]
            if path_length > 2:
                self.poi_last = self.new_path[2]

                # check direction
                if (self.prev_path[1] is not self.poi_start[0]):
                    if (self.prev_path[0] is not self.poi_start[0]):
                        # you have gone the wrong way
                        self.turn = self.turn_around
                        
        
                self.prev_path = self.new_path
                # from 0 to 1 indluded
                for index in range(2):
                    self.displacement[index] = self.graph[(self.poi_next, self.poi_last)][index] - self.graph[(self.poi_start, self.poi_next)][index]
                    if index == 1:
                        dispx = self.displacement[index-1]
                        dispy = self.displacement[index]
                        if dispx == 0 or dispy == 0:
                            self.turn = self.forward
                        elif dispx > 0 and dispy > 0:
                            self.turn = self.left
                            
                        elif dispx > 0 and dispy < 0:
                            self.turn = self.right
                        elif dispx < 0 and dispy > 0:
                            self.turn = self.right
                        elif dispx < 0 and dispy < 0:
                            self.turn = self.left

            # Path has two points: if they are the same, destination has been reached
            elif path_length == 2:
                # one case is that you have arrived
                if self.poi_next == self.poi_start:
                    self.turn = self.arrived
                # the other case worth noting is when the second value (i.e.destination)
                # from the old path is not the same as the destination in the new path
                elif self.poi_next is not self.prev_path[1]:
                    self.turn = self.turn_around
                # the third circumstance is when the user hadn't moved and re-scanned the node
                else:
                    self.turn = self.forward

        # there is only one or no values, which should not happen
        else:
            if path_length == 1 and self.poi_start == self.prev_path[0]:
                self.turn = self.arrived

            else:
                self.turn = self.choose_dest

        return self.turn

    ## Process Orientation
    #  sets self.turn to give feedback to user based on magnetometer readings
    #  @param [in] self The object pointer.
    #  @param [in] ax_under Normalized magnetometer projection onto x-axis
    #  @param [in] ay_under Normalized magnetometer projection onto y-axis                
    def check_turn(self):
        if self.turn == self.arrived:
            return True

    ## Direct TTS
    #  sets self.turn to give feedback to user based on magnetometer readings
    #  @param [in] self The object pointer.
    def direct_tts(self):
        if self.turn == self.left:
            os.system(self.turn_left_tts)

        elif self.turn == self.right:
            os.system(self.turn_right_tts)
            
        elif self.turn == self.arrived:
            os.system(self.arrived_tts)
        
        elif self.turn == self.turn_around:
            os.system(self.turn_around_tts)
        
        else:
            return

    ## Process Turn
    #  sets self.turn to give feedback to user based on magnetometer readings
    #  @param [in] self The object pointer.
    def process_turn(self):
        
        # create waveform packets of 5
        if self.turn == self.left:
            self.effect = [44, 71, 44, 71, 44, 0]

        elif self.turn == self.right:
            self.effect = [44, 82, 44, 82, 44, 0]
            
        elif self.turn == self.arrived:
            self.effect = [82, 71, 82, 71, 0]
        
        elif self.turn == self.turn_around:
            self.effect = [14, 14, 14, 14, 0]
        
        elif self.turn == self.forward:
            self.effect = [0]
            
    ## Process Orientation
    #  sets self.turn to give feedback to user based on magnetometer readings
    #  @param [in] self The object pointer.
    #  @param [in] ax_under Normalized magnetometer projection onto x-axis
    #  @param [in] ay_under Normalized magnetometer projection onto y-axis
    def processOrientation(self, ax_under, ay_under):
        # as long as self.new_path has two values, check if feedback is needed
        if (len(self.new_path) > 1):
            # check array to see if values are in self.graph. if not, we're done here
            if (self.new_path[0], self.new_path[1]) in self.graph:
                # otherswise, assign values to local variables
                x_val = self.graph[(self.new_path[0], self.new_path[1])][0]
                y_val = self.graph[(self.new_path[0], self.new_path[1])][1]

                # compare magnet reading to when next node is in positive y-direction
                if x_val == 0 and y_val > 0:
                    if ax_under > -0.125 or ax_under < 0.125 and ay_under < 0:
                        self.turn = self.turn_around
                    elif ax_under > 0 and ay_under < 0 :
                        self.turn = self.left
                    elif ax_under < 0 and ay_under < 0: 
                        self.turn = self.right
                    else:
                        self.turn = self.forward

                # compare magnet reading to when next node is in the negative y-direction
                elif x_val == 0 and y_val < 0:
                    if ax_under > -0.125 or ax_under < 0.125 and ay_under > 0:
                        self.turn = self.turn_around
                    elif ax_under > 0 and ay_under > 0 :
                        self.turn = self.right
                    elif ax_under < 0 and ay_under > 0: 
                        self.turn = self.left
                    else:
                        self.turn = self.forward
                        
                # compare magnet reading to when next node is in the positive x-direction
                elif x_val > 0 and y_val == 0:
                    if ay_under > -0.125 or ay_under < 0.125 and ax_under < 0:
                        self.turn = self.turn_around
                    elif ay_under > 0 and ax_under < 0 :
                        self.turn = self.right
                    elif ay_under < 0 and ax_under < 0: 
                        self.turn = self.left
                    else:
                        self.turn = self.forward
                        
                # compare magnet reading to when next node is in the negative x-direction
                elif x_val < 0 and y_val == 0:
                    if ay_under > -0.125 or ay_under < 0.125 and ax_under > 0:
                        self.turn = self.turn_around
                    elif ay_under > 0 and ax_under > 0 :
                        self.turn = self.left
                    elif ay_under < 0 and ax_under > 0: 
                        self.turn = self.right
                    else:
                        self.turn = self.forward


import smbus
import time
import math


## MPU9250 Default I2C slave address
SLAVE_ADDRESS        = 0x68
## AK8963 I2C slave address
AK8963_SLAVE_ADDRESS = 0x0C
## Device id
DEVICE_ID            = 0x71

## sample rate driver
SELF_TEST_X_GYRO = 0x0
SELF_TEST_Y_GYRO = 0x1
SELF_TEST_Z_GYRO = 0x2
SELF_TEST_X_ACCEL = 0xD
SELF_TEST_Y_ACCEL = 0xE
SELF_TEST_Z_ACCEL = 0xF
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
LP_ACCEL_ODR   = 0x1E
WOM_THR        = 0x1F
FIFO_EN        = 0x23
I2C_MST_CTRL   = 0x24
I2C_MST_STATUS = 0x36
INT_PIN_CFG    = 0x37
INT_ENABLE     = 0x38
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43

XG_OFFSET_H    = 0x13
XG_OFFSET_L    = 0x14
YG_OFFSET_H    = 0x15
YG_OFFSET_L    = 0x16
ZG_OFFSET_H    = 0x17
ZG_OFFSET_L    = 0x18
ACCEL_XOUT_H   = 0x3B
ACCEL_XOUT_L   = 0x3C
ACCEL_YOUT_H   = 0x3D
ACCEL_YOUT_L   = 0x3E
ACCEL_ZOUT_H   = 0x3F
GYRO_XOUT_H    = 0x43
GYRO_XOUT_H    = 0x44
GYRO_XOUT_H    = 0x45
GYRO_XOUT_H    = 0x46
GYRO_XOUT_H    = 0x47
GYRO_XOUT_H    = 0x48
ACCEL_ZOUT_L   = 0x40
XA_OFFSET_H    = 0x77
XA_OFFSET_L    = 0x78
YA_OFFSET_H    = 0x7A
YA_OFFSET_L    = 0x7B
ZA_OFFSET_H    = 0x7D
ZA_OFFSET_L    = 0x7E

I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET  = 0x68
MOT_DETECT_CTRL    = 0x69
USER_CTRL          = 0x6A
PWR_MGMT_1         = 0x6B
PWR_MGMT_2         = 0x6C
FIFO_COUNTH        = 0x72
FIFO_R_W           = 0x74
WHO_AM_I           = 0x75

## Gyro Full Scale Select 250dps
GFS_250  = 0x00
## Gyro Full Scale Select 500dps
GFS_500  = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000 = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000 = 0x03
## Accel Full Scale Select 2G
AFS_2G   = 0x00
## Accel Full Scale Select 4G
AFS_4G   = 0x01
## Accel Full Scale Select 8G
AFS_8G   = 0x02
## Accel Full Scale Select 16G
AFS_16G  = 0x03

# AK8963 Register Addresses
AK8963_ST1        = 0x02
AK8963_MAGNET_OUT = 0x03
AK8963_CNTL1      = 0x0A
AK8963_CNTL2      = 0x0B
AK8963_ASAX       = 0x10

# CNTL1 Mode select
## Power down mode
AK8963_MODE_DOWN   = 0x00
## One shot data output
AK8963_MODE_ONE    = 0x01

## Continous data output 8Hz
AK8963_MODE_C8HZ   = 0x02
## Continous data output 100Hz
AK8963_MODE_C100HZ = 0x06

# Magneto Scale Select
## 14bit output
AK8963_BIT_14 = 0x00
## 16bit output
AK8963_BIT_16 = 0x01

# Pin definitions
intPin = 12 # These can be changed, 2 and 3 are the Arduinos ext int pins

accelCount = [0.0, 0.0, 0.0]# Stores the 16-bit signed accelerometer sensor output
gyroCount = [0.0, 0.0, 0.0] # Stores the 16-bit signed gyro sensor output
magCount = [0.0, 0.0, 0.0]# Stores the 16-bit signed magnetometer sensor output
magCalibration = [0.0, 0.0, 0.0] 
magbias = [0.0, 0.0, 0.0] # Factory mag calibration and mag bias
gyroBias = [0.0, 0.0, 0.0] 
accelBias = [0.0, 0.0, 0.0] # Bias corrections for gyro and accelerometer
ax, ay, az, gx, gy, gz, mx, my, mz = (0, 0, 0, 0, 0, 0, 0, 0, 0) # variables to hold latest sensor data values 
tempCount = 0 # Stores the real internal chip temperature in degrees Celsius
temperature = 0.0
SelfTest = [0.0,0.0,0.0,0.0,0.0,0.0]

delt_t = 0 # used to control display output rate
count = 0 # used to control display output rate

# parameters for 6 DoF sensor fusion calculations
GyroMeasError = math.pi * (60.0 / 180.0)     # gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
beta = math.sqrt(3.0 / 4.0) * GyroMeasError  # compute beta
GyroMeasDrift = math.pi * (1.0 / 180.0) #gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
zeta = math.sqrt(3.0 / 4.0) * GyroMeasDrift  # compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
# Check values for Kp and Ki, as the documentation commented out these variables with the values assigned below
Kp = 2.0 * 10.0     #define Kp 2.0 * 5.0 # these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
Ki = 0.0            #define Ki 0.0

pitch, yaw, roll = (0.0, 0.0, 0.0)  # Instantiation for quaterion-calculated ptch yaw roll
deltat = 0.0                      # integration interval for both filter schemes
lastUpdate = 0  			# used to calculate integration interval 
firstUpdate = 0  			# used to calculate integration interval 
Now = 0                            	# used to calculate integration interval                             
q = [1.0, 0.0, 0.0, 0.0]     	# vector to hold quaternion
eInt = [0.0, 0.0, 0.0]             	# vector to hold integral error for Mahony method

# declare smbus
bus = smbus.SMBus(1)

## MPU9250 I2C Controll class
class MPU9250:
# use for simple checks

    ## Constructor
    #  @param [in] address MPU-9250 I2C slave address default:0x68
    def __init__(self, address=SLAVE_ADDRESS):
        self.address = address
        self.configMPU9250(GFS_250, AFS_2G)
        self.configAK8963(AK8963_MODE_C8HZ, AK8963_BIT_16)
        self.length = 0.0
        self.v1 = [0,0,0]
        self.v2 = [0,0,0]
        self.b = [0,0,0]

    ## Search Device
    #  @param [in] self The object pointer.
    #  @retval true device connected
    #  @retval false device error
    def searchDevice(self):
        who_am_i = bus.read_byte_data(self.address, WHO_AM_I)
        if(who_am_i == DEVICE_ID):
            return True
        else:
            return False

    ## Configure MPU-9250
    #  @param [in] self The object pointer.
    #  @param [in] gfs Gyro Full Scale Select(default:GFS_250[+250dps])
    #  @param [in] afs Accel Full Scale Select(default:AFS_2G[2g])
    def configMPU9250(self, gfs, afs):
        if gfs == GFS_250:
            self.gres = 250.0/32768.0
        elif gfs == GFS_500:
            self.gres = 500.0/32768.0
        elif gfs == GFS_1000:
            self.gres = 1000.0/32768.0
        else:  # gfs == GFS_2000
            self.gres = 2000.0/32768.0

        if afs == AFS_2G:
            self.ares = 2.0/32768.0
        elif afs == AFS_4G:
            self.ares = 4.0/32768.0
        elif afs == AFS_8G:
            self.ares = 8.0/32768.0
        else: # afs == AFS_16G:
            self.ares = 16.0/32768.0

        # sleep off
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # auto select clock source
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # DigitalLowPassFilter_CFG enabled means 1kHz sample rate for Gyro (Acc is 1kHz anyway)
        bus.write_byte_data(self.address, CONFIG, 0x03)
        # sample rate divider. 0X4 sets the rate to 200 Hz for Gyro/Acc
        bus.write_byte_data(self.address, SMPLRT_DIV, 0x04)
        # gyro full scale select from FS_SEL and AFS_SEL
        bus.write_byte_data(self.address, GYRO_CONFIG, gfs << 3)
        # accel full scale select
        bus.write_byte_data(self.address, ACCEL_CONFIG, afs << 3)
        # A_DLPFCFG
        bus.write_byte_data(self.address, ACCEL_CONFIG_2, 0x03)
        # BYPASS_EN
        bus.write_byte_data(self.address, INT_PIN_CFG, 0x02)
        time.sleep(0.1)

    ## Configure AK8963
    #  @param [in] self The object pointer.
    #  @param [in] mode Magneto Mode Select(default:AK8963_MODE_C8HZ[Continous 8Hz])
    #  @param [in] mfs Magneto Scale Select(default:AK8963_BIT_16[16bit])
    def configAK8963(self, mode, mfs):
        if mfs == AK8963_BIT_14:
            self.mres = 4912.0/8190.0 #multiply by 10 to get milliGauss 
        else: #  mfs == AK8963_BIT_16: 
            self.mres = 4912.0/32760.0  #multiply by 10 to get milliGauss

        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.01)

        # set read FuseROM mode
        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x0F)
        time.sleep(0.01)

        # read coef data, 6 bytes
        data = bus.read_i2c_block_data(AK8963_SLAVE_ADDRESS, AK8963_ASAX, 3)

        self.magXcoef = (data[0] - 128) / 256.0 + 1.0
        self.magYcoef = (data[1] - 128) / 256.0 + 1.0
        self.magZcoef = (data[2] - 128) / 256.0 + 1.0

        # set power down mode
        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.01)

        # set scale&continous mode. mfs 16 bit (14 bit) at 1 (0) and mode is 8Hz (100Hz) at 0x2(0x6)
        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, (mfs<<4|mode))
        time.sleep(0.01)

    ## brief Check data ready
    #  @param [in] self The object pointer.
    #  @retval true data is ready
    #  @retval false data is not ready
    def checkDataReady(self):
        drdy = bus.read_byte_data(self.address, INT_STATUS)
        if drdy & 0x01:
            return True
        else:
            return False

    ## Read accelerometer
    #  @param [in] self The object pointer.
    #  @retval x : x-axis data
    #  @retval y : y-axis data
    #  @retval z : z-axis data
    def readAccel(self):
        data = bus.read_i2c_block_data(self.address, ACCEL_OUT, 6)
        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.ares, 3)
        y = round(y*self.ares, 3)
        z = round(z*self.ares, 3)

        return [x, y, z]

    ## Read gyro
    #  @param [in] self The object pointer.
    #  @retval x : x-gyro data
    #  @retval y : y-gyro data
    #  @retval z : z-gyro data
    def readGyro(self):
        data = bus.read_i2c_block_data(self.address, GYRO_OUT, 6)

        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.gres, 3)
        y = round(y*self.gres, 3)
        z = round(z*self.gres, 3)

        return [x, y, z]

    ## Read magneto
    #  @param [in] self The object pointer.
    #  @retval x : X-magneto data
    #  @retval y : y-magneto data
    #  @retval z : Z-magneto data
    def readMagnet(self):
        x=0
        y=0
        z=0

        # check data ready
        drdy = bus.read_byte_data(AK8963_SLAVE_ADDRESS, AK8963_ST1)
        if drdy & 0x01 :
            data = bus.read_i2c_block_data(AK8963_SLAVE_ADDRESS, AK8963_MAGNET_OUT, 7)

            # check overflow
            if (data[6] & 0x08)!=0x08:
                x = self.dataConv(data[0], data[1])
                y = self.dataConv(data[2], data[3])
                z = self.dataConv(data[4], data[5])

                x = round(x * self.mres * self.magXcoef, 3)
                y = round(y * self.mres * self.magYcoef, 3)
                z = round(z * self.mres * self.magZcoef, 3)

        return [x, y, z]

    ## Read temperature
    #  @param [out] temperature temperature(degrees C)
    def readTemperature(self):
        data = bus.read_i2c_block_data(self.address, TEMP_OUT, 2)
        temp = self.dataConv(data[1], data[0])

        temp = round((temp / 333.87 + 21.0), 3)
        return temp

    ## Data Convert
    # @param [in] self The object pointer.
    # @param [in] data1 LSB
    # @param [in] data2 MSB
    # @retval Value MSB+LSB(int 16bit)
    def dataConv(self, data1, data2):
        value = data1 | (data2 << 8)
        if(value & (1 << 16 - 1)):
            value -= (1<<16)
        return value

    # Callibrate 
    # Accumulates gyro/acc data after device initialization. It calculates average 
    # of the at-rest readins and loads resulting offsets into acc/gyro bias regs
    def biasConfig(self, dest1, dest2):
        holdData=[]
        accel_bias=[0,0,0]
        gyro_bias = [0,0,0]
        
        #reset device
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x80)   # resets device
        time.sleep(0.02)
        
        #set stale tim source to be PLL w/x-axis gyro referece (0x1)
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x1)  
        bus.write_byte_data(self.address, PWR_MGMT_2,0x2)
        time.sleep(0.2)
        
        #entire device configuration
        bus.write_byte_data(self.address, INT_ENABLE, 0x0)      #disable all interrupts
        bus.write_byte_data(self.address, FIFO_EN, 0x0)         #disable FIFO
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x0)      #use internal clock source
        bus.write_byte_data(self.address, I2C_MST_CTRL, 0x0)    #disable i2c master
        bus.write_byte_data(self.address, USER_CTRL, 0x0)       #disable fifo and i2c master modes
        bus.write_byte_data(self.address, USER_CTRL, 0xC)       #reset fifo and dmp
        time.sleep(0.02)
        
        #Acc/Gyro configuration for bias
        bus.write_byte_data(self.address, CONFIG, 0x1)          #set low-pass filter to 188Hz
        bus.write_byte_data(self.address, SMPLRT_DIV, 0x0)      #set sample rate to 1kHz
        bus.write_byte_data(self.address, GYRO_CONFIG, 0x0)     #set gyro full-scale to 250 degrees/sec, the max sensitivity
        bus.write_byte_data(self.address, ACCEL_CONFIG, 0x0)    #set acc full-scal to 2g, the max sensitivity
        
        gyroSensitivity=131     # in LSB/(deg*sec)
        accSensitivity=16384    # in LSB/g
        
        #Config FIFO to capture accelerometer and gyro data for bias calculation
        bus.write_byte_data(self.address, USER_CTRL, 0x40)  #enable FIFO
        bus.write_byte_data(self.address, FIFO_EN, 0x78)    #enable gyro and acc sensors for FIFO (max size 512 bytes in MPU250)
        time.sleep(0.04)                            #40 samples in 80ms is 480 bytes
        
        #Turn off FIFO after sample accumulation after 
        bus.write_byte_data(self.address, FIFO_EN, 0x0)     #FIFO off
        holdData = bus.read_byte_data(self.address, FIFO_COUNTH, 2) #store data from FIFO_COUNTH reg (2 bytes)
        fifo_count = self.dataConv(holdData[1], holdData[0])    #store data in readable forat
        packet_count = fifo_count/12        # how many sets of full gyro and acc data will be averaged
        
        for average in range (packet_count):
            accel_temp=[0,0,0]
            gyro_temp=[0,0,0]
            holdData = bus.read_byte_data(self.address, FIFO_R_W, 12)   # read data for averaging
            accel_temp[0]=self.dataConv(holdData[1], holdData[0])           # From signed 16 bit integer for each sample in FIFO
            accel_temp[1]=self.dataConv(holdData[3], holdData[2])           # From signed 16 bit integer for each sample in FIFO
            accel_temp[2]=self.dataConv(holdData[5], holdData[4])           # From signed 16 bit integer for each sample in FIFO
            gyro_temp[0]=self.dataConv(holdData[7], holdData[6])           # From signed 16 bit integer for each sample in FIFO
            gyro_temp[1]=self.dataConv(holdData[9], holdData[8])           # From signed 16 bit integer for each sample in FIFO
            gyro_temp[2]=self.dataConv(holdData[11], holdData[10])           # From signed 16 bit integer for each sample in FIFO
            
            #accumulate readings
            accel_bias[0]=accel_bias[0] + accel_temp[0]             
            accel_bias[1]=accel_bias[1] + accel_temp[1]
            accel_bias[2]=accel_bias[2] + accel_temp[2]
            gyro_bias[0]=accel_bias[0] + accel_temp[0]
            gyro_bias[1]=accel_bias[1] + accel_temp[1]
            gyro_bias[2]=accel_bias[2] + accel_temp[2]
            
        #normalize sums to get average count bias
        accel_bias[0] /= packet_count
        accel_bias[1] /= packet_count
        accel_bias[2] /= packet_count
        gyro_bias[0] /= packet_count
        gyro_bias[1] /= packet_count
        gyro_bias[2] /= packet_count        
            
        if accel_bias[2]>0:
            accel_bias[2]=accel_bias-accSensitivity     #remove gravity from the z-axis acc bias calculation
        else:
            accel_bias[2]=accel_bias[2]+accSensitivity  
        
        #construct Gyro  biases for pushing the gyro to the hardware gyro bias registers. They are reset to zero upon device startup
        holdData[0] = (-gyro_bias[0]/4 >> 8) & 0xFF     # Divide by 4 to get 32.9 LSB/(deg*sec) to conform to expeced bias input format
        holdData[1] =  -gyro_bias[0]/4       & 0xFF     # Biases are additive, so change the sign on each calculated gyro bias
        holdData[2] = (-gyro_bias[0]/4 >> 8) & 0xFF     
        holdData[3] =  -gyro_bias[0]/4       & 0xFF     
        holdData[4] = (-gyro_bias[0]/4 >> 8) & 0xFF     
        holdData[5] =  -gyro_bias[0]/4       & 0xFF             
        
#     Uncommenting this block of code will change hardware charactoristics and runs a high risk of future data collection to be wrong
#
#
#        # Push gyro biases to hardware registers
#        bus.write_byte_data(self.address, XG_OFFSET_H, holdData[0])
#        bus.write_byte_data(self.address, XG_OFFSET_L, holdData[1])
#        bus.write_byte_data(self.address, YG_OFFSET_H, holdData[2])
#        bus.write_byte_data(self.address, YG_OFFSET_L, holdData[3])
#        bus.write_byte_data(self.address, ZG_OFFSET_H, holdData[4])
#        bus.write_byte_data(self.address, ZG_OFFSET_L, holdData[5])
#
        #Construct gyro ias in deg/s for later manual subtraction
        dest1[0] = gyro_bias[0]/gyroSensitivity    
        dest1[1] = gyro_bias[1]/gyroSensitivity            
        dest1[2] = gyro_bias[2]/gyroSensitivity            


# Construct the accelerometer biases for pushing onto the hardwar accelerometer bias registers. 
# These registers contain factory trim values which must be added to the calculated biases;
# on bootup, these registers will hold non0zero values. In addition, bit 0 of the lower byte
# must be preserved since it is used for temperature compensation calculations. 
# Accelerometer bias registers exect bias input as 2048 LSB/g, so that the acceleromter
# biases calcuated above must b divided by 8 (from 16384)

        mask=1                  #Defin mask for temperature compensation bit 0 of lower byt of acc bias reg
        mask_bit=[0,0,0]        # Define aray to hold mask bit for each acc bias axis
        accel_bias_reg=[0,0,0]  #A place to hold the factory trim biases

        holdData = bus.read_byte_data(self.address,XA_OFFSET_H,2)     #Read factory data into holdData
        accel_bias_reg[0] = self.dataConv(holdData[1], holdData[0])
        holdData = bus.read_byte_data(self.address,YA_OFFSET_H,2)     #Read factory data into holdData
        accel_bias_reg[0] = self.dataConv(holdData[1], holdData[0])
        holdData = bus.read_byte_data(self.address,ZA_OFFSET_H,2)     #Read factory data into holdData
        accel_bias_reg[0] = self.dataConv(holdData[1], holdData[0])

        for eachDataReg in range (3):
            if accel_bias_reg[eachDataReg] & mask:
                mask_bit[eachDataReg] = 0x1         #If temp compensation mask is set, make a note in mask_bit

        # Construct total accelerometer bias, including calculated average acceleromter  bias from above
        accel_bias_reg[0] = accel_bias_reg[0] - accel_bias[0]/8    # Subtract calculated aceragedaccelerometer bias scaled to 2048
        accel_bias_reg[1] = accel_bias_reg[1] - accel_bias[1]/8
        accel_bias_reg[2] = accel_bias_reg[2] - accel_bias[2]/8        

        holdData[0] = (accel_bias_reg[0]/4 >> 8) & 0xFF     # Divide by 4 to get 32.9 LSB/(deg*sec) to conform to expeced bias input format
        holdData[1] =  accel_bias_reg[0]/4       & 0xFF     # Biases are additive, so change the sign on each calculated gyro bias
        holdData[1] = holdData[1] | mask_bit[0]             # Preserve temperature compensation bit when writing back to acc bias registers
        holdData[2] = (accel_bias_reg[0]/4 >> 8) & 0xFF     
        holdData[3] =  accel_bias_reg[0]/4       & 0xFF     
        holdData[1] = holdData[3] | mask_bit[1]
        holdData[4] = (accel_bias_reg[0]/4 >> 8) & 0xFF     
        holdData[5] =  accel_bias_reg[0]/4       & 0xFF                 
        holdData[1] = holdData[5] | mask_bit[2]
                                
#     Uncommenting this block of code will change hardware charactoristics and runs a high risk of future data collection to be wrong
#
#
#        # This is not working for the accelerometer bises in the MPU-9250
#        # Check handling of temperature preservation bit
#        
#        # Push accelerometer biases to hardware registers        
#        bus.write_byte_data(self.address, XA_OFFSET_H, holdData[0])
#        bus.write_byte_data(self.address, XA_OFFSET_L, holdData[1])
#        bus.write_byte_data(self.address, YA_OFFSET_H, holdData[2])
#        bus.write_byte_data(self.address, YA_OFFSET_L, holdData[3])
#        bus.write_byte_data(self.address, ZA_OFFSET_H, holdData[4])
#        bus.write_byte_data(self.address, ZA_OFFSET_L, holdData[5])
#
        dest2[0] = accel_bias[0] / accSensitivity
        dest2[1] = accel_bias[1] / accSensitivity
        dest2[2] = accel_bias[2] / accSensitivity
 
    # Accelerometer and Gyroscope Self-Test
    # Checks calibration of factory settings
    # should return %dev from factory trim values, +/- 14 to pass
    def MPU9250SelfTest(self, destination):
        rawData = [0,0,0,0,0,0]
        selfTest = [0,0,0,0,0,0]
        gAvg = [0,0,0]
        aAvg = [0,0,0]
        gSTAvg = [0,0,0]
        aSTAvg = [0,0,0]
        factoryTrim = [0,0,0,0,0,0]
        FS = 0

        bus.write_byte_data(self.address, SMPLRT_DIV, 0x0)  # Set gyro sample rate to 1kHz
        bus.write_byte_data(self.address, CONFIG, 0x2)  # Set gyro sample rate to 1kHz 
        bus.write_byte_data(self.address, GYRO_CONFIG, FS<<3)  # Set gyro sample rate to 1kHz
        bus.write_byte_data(self.address, ACCEL_CONFIG, 0x2)  # Set gyro sample rate to 1kHz
        bus.write_byte_data(self.address, ACCEL_CONFIG_2, FS<<3)  # Set gyro sample rate to 1kHz                        
            
        # Get average current values of gyro and accelerometer            
        for singleValue in range (200):
            #Read the six raw data accelerometer registers into raw data array
            rawData = bus.read_byte_data(self.address, ACCEL_XOUT_H, 6)
            #Turn MSB an LSB into signed 16-bit value
            aAvg[0] = aAvg[0] + self.dataConv(rawData[1], rawData[0])   
            aAvg[1] = aAvg[1] + self.dataConv(rawData[3], rawData[2])   
            aAvg[2] = aAvg[2] + self.dataConv(rawData[5], rawData[4])
            
            #Read the six raw data gyroscope registers into raw data array
            rawData = bus.read_byte_data(self.address, GYRO_XOUT_H, 6)
            #Turn MSB an LSB into signed 16-bit value
            gAvg[0] = gAvg[0] + self.dataConv(rawData[1], rawData[0])   
            gAvg[1] = gAvg[1] + self.dataConv(rawData[3], rawData[2])   
            gAvg[2] = gAvg[2] + self.dataConv(rawData[5], rawData[4])
            
        # Take 200 measurements fom above and average them 
        for takeAve in range (3):
            aAvg[takeAve] /= 200
            gAvg[takeAve] /= 200

        #configure the accelerometer for self-test
        bus.write_byte_data(self.address, ACCEL_CONFIG, 0xE0)   #Enable self-test on all 3 axes and set acc range to +/-2g
        bus.write_byte_data(self.address, GYRO_CONFIG, 0xE0)    #Enable self-test on all 3 axes and set gyro range to +/-250 deg/s
        time.sleep(25) #delay for some time
        
        for getAveVal in range (200):
            # Read the six accelerometer data registers, aving them into aSTAvg 
            rawData = bus.read_byte_data(self.address, ACCEL_XOUT_H, 6)
            aSTAvg[0] = aSTAvg[0] + self.dataConv(rawData[1], rawData[0])
            aSTAvg[1] = aSTAvg[1] + self.dataConv(rawData[3], rawData[2])
            aSTAvg[2] = aSTAvg[2] + self.dataConv(rawData[5], rawData[4])        

            # Read the six gyroscope data registers, aving them into aSTAvg 
            rawData = bus.read_byte_data(self.address, GYRO_XOUT_H, 6)
            # Turn the MSB and LSB into a signed 16-bit value            
            gSTAvg[0] = gSTAvg[0] + self.dataConv(rawData[1], rawData[0])
            gSTAvg[1] = gSTAvg[1] + self.dataConv(rawData[3], rawData[2])
            gSTAvg[2] = gSTAvg[2] + self.dataConv(rawData[5], rawData[4])        
            
        # Calculate averages in gATDev
        for computeAve in range (3):
            aSTAvg[computeAve] /= 200
            gSTAvg[computeAve] /= 200
            
        # Retrieve factory self0test value from self0test code reads
        selfTest[0] = bus.read_byte_data(self.address, SELF_TEST_X_ACCEL) # X-axis accel self-tset results
        selfTest[1] = bus.read_byte_data(self.address, SELF_TEST_Y_ACCEL) # Y-axis accel self-tset results
        selfTest[2] = bus.read_byte_data(self.address, SELF_TEST_Z_ACCEL) # Z-axis accel self-tset results
        selfTest[3] = bus.read_byte_data(self.address, SELF_TEST_X_GYRO) # X-axis gyro self-tset results
        selfTest[4] = bus.read_byte_data(self.address, SELF_TEST_Y_GYRO) # Y-axis gyro self-tset results
        selfTest[5] = bus.read_byte_data(self.address, SELF_TEST_Z_GYRO) # Z-axis gyro self-tset results            
           
        # Retrieve factory self-test value from self-test code reads
        factoryTrim[0] = 2620*FS*pow(1.01, (selfTest[0]-1.0))  #FT[Xa] factory trim calculation
        factoryTrim[1] = 2620*FS*pow(1.01, (selfTest[1]-1.0))  #FT[Xa] factory trim calculation
        factoryTrim[2] = 2620*FS*pow(1.01, (selfTest[2]-1.0))  #FT[Xa] factory trim calculation
        factoryTrim[3] = 2620*FS*pow(1.01, (selfTest[3]-1.0))  #FT[Xa] factory trim calculation
        factoryTrim[4] = 2620*FS*pow(1.01, (selfTest[4]-1.0))  #FT[Xa] factory trim calculation
        factoryTrim[5] = 2620*FS*pow(1.01, (selfTest[5]-1.0))  #FT[Xa] factory trim calculation
           
        # Report results as a ratio of (STR -FT/FT; the change from Factory Trim of the Self0Test Response
        for eachDataReg in range (3):
            destination[eachDataReg]=100.0*(aSTAvg[eachDataReg]-aAvg[eachDataReg])/factoryTrim[eachDataReg]-100     #Report Percent Differences
            destination[eachDataReg+3]=100.0*(aSTAvg[eachDataReg]-aAvg[eachDataReg])/factoryTrim[eachDataReg+3]-100     #Report Percent Differences
           
    # Quaternian Calculations
    # Sebastian Madgwick's @... efficient orinetation filter for... inertial/ magnetic sensor arrays@
    # 
    # Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
    # (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
    # which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
    # device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    # The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    # but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
    def MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz):
        
        q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]   # short name local variable for readability
        norm = 0.0
        hx, hy, _2bx, _2bz = (0.0, 0.0, 0.0, 0.0)
        s1, s2, s3, s4 = (0.0, 0.0, 0.0, 0.0)
        qDot1, qDot2, qDot3, qDot4 = (0.0, 0.0, 0.0, 0.0)

        # Auxiliary variables to avoid repeated arithmetic
        _2q1mx = 0.0
        _2q1my = 0.0
        _2q1mz = 0.0
        _2q2mx = 0.0
        _4bx = 0.0
        _4bz = 0.0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        #Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:
            return      # handle NaN
        norm = 1.0/norm
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm == 0.0:
            return      # handle NaN
        norm = 1.0/norm
        mx *= norm
        my *= norm
        mz *= norm

        #Reference direction of Earth's magnetic field
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)   # normalise step magnitude
        norm = 1.0/norm
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        norm = 1.0/norm
        q[0] = q1 * norm
        q[1] = q2 * norm
        q[2] = q3 * norm
        q[3] = q4 * norm
  
      # Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
      # measured ones.
    def MahonyQuaternionUpdate(ax,  ay,  az, gx, gy, gz, mx, my, mz):
    
        q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]   # short name local variable for readability
        norm = 0.0
        hx, hy, bx, bz = (0.0, 0.0, 0.0, 0.0)
        vx, vy, vz, wx, wy, wz = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        ex, ey, ez = (0.0, 0.0, 0.0)
        pa, pb, pc = (0.0, 0.0, 0.0)

        # Auxiliary variables to avoid repeated arithmetic
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4   

        # Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az);
        if norm == 0.0:
            return; # handle NaN
        norm = 1.0 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz);
        if norm == 0.0: 
            return # handle NaN
        norm = 1.0 / norm        # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        #Reference direction of Earth's magnetic field
        hx = 2.0 * mx * (0.5 - q3q3 - q4q4) + 2.0 * my * (q2q3 - q1q4) + 2.0 * mz * (q2q4 + q1q3)
        hy = 2.0 * mx * (q2q3 + q1q4) + 2.0 * my * (0.5 - q2q2 - q4q4) + 2.0 * mz * (q3q4 - q1q2)
        bx = math.sqrt((hx * hx) + (hy * hy))
        bz = 2.0 * mx * (q2q4 - q1q3) + 2.0 * my * (q3q4 + q1q2) + 2.0 * mz * (0.5 - q2q2 - q3q3)

        # Estimated direction of gravity and magnetic field
        vx = 2.0 * (q2q4 - q1q3)
        vy = 2.0 * (q1q2 + q3q4)
        vz = q1q1 - q2q2 - q3q3 + q4q4
        wx = 2.0 * bx * (0.5 - q3q3 - q4q4) + 2.0 * bz * (q2q4 - q1q3)
        wy = 2.0 * bx * (q2q3 - q1q4) + 2.0 * bz * (q1q2 + q3q4)
        wz = 2.0 * bx * (q1q3 + q2q4) + 2.0 * bz * (0.5 - q2q2 - q3q3)  

        # Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        if Ki > 0.0:
            eInt[0] += ex      # accumulate integral error
            eInt[1] += ey
            eInt[2] += ez
        else:
            eInt[0] = 0.0     # prevent integral wind up
            eInt[1] = 0.0
            eInt[2] = 0.0

        # Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0]
        gy = gy + Kp * ey + Ki * eInt[1]
        gz = gz + Kp * ez + Ki * eInt[2]

        # Integrate rate of change of quaternion
        pa = q2
        pb = q3
        pc = q4
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * deltat)
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 * deltat)
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 * deltat)
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 * deltat)

        # Normalise quaternion
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        norm = 1.0 / norm
        q[0] = q1 * norm
        q[1] = q2 * norm
        q[2] = q3 * norm
        q[3] = q4 * norm
  
    # Read IMU
    # Returns averages of samples over a 5-second interval, 
    # averageReadings returns the values in the following order:
    #   [xAcc, yAcc, zAcc, xGy, yGy, zGy, xMag, yMag, zMag, temp]
    # the other returned value is a float, but can be typcast into an int for memory optimization
    # @param [in] self The object pointer.
    # @param [in] readAcc (boolean)
    # @param [in] readGyro (boolean)
    # @param [in] readMag (boolean)
    # @param [in] getMagnitude (boolean)
    # @param [in] getVelocity (boolean)
    # @retval averageReadings (float), total_sample_time (int), magMag (float), speed (float)
    def readIMU(self, readAcc, readGyro, readMag, getMagnitude, getVelocity):
        
        #Select proper delay time for reading data based on sensor settings. Double delay to increase data reliability
        sample_delay = 2/(self.gres*32760)      # Long wait to ensure more accurate data 
        total_samples = 1/(2 * sample_delay)    # Total number of samples to divide. If sample_delay is not multiplied, sample time is 1 sec, which is too slow 
        acc_gyro_samples = total_samples/20     # The magnetometer being the bottle neck will alway sampl 20 times. Gyro/Acc can be sampled quicker for more accurate data
        total_sample_time = sample_delay + total_samples  # Not necessary, as this should always be 0.5 (seconds)
        averageReadings=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        readTemp=False

        # someone called the function without passing any values 
        if readAcc is False and readGyro is False and readMag is False:
            readTemp = True

        elif readMag is True:                
            # set gyro and acc as nested for loop, assuming count starts at 0 and goes to 19 (20 total readings). Time so by the ith reading, the mag is ready
            for QUICK_READ_ALL in range (20):
                if readAcc is True or readGyro is True:
                    #Starting at 1 and going to 40 (40 readings)
                    for ACC_GYRO in range (int(acc_gyro_samples)):
                        if readAcc is True:    
                            accel = MPU9250.readAccel(self)
                            averageReadings[0] = averageReadings[0] + accel['x']
                            averageReadings[1] = averageReadings[1] + accel['y']
                            averageReadings[2] = averageReadings[2] + accel['z']

                        if readGyro is True:
                            gyro = MPU9250.readGyro(self)
                            averageReadings[3] = averageReadings[3] + gyro['x']
                            averageReadings[4] = averageReadings[4] + gyro['y']
                            averageReadings[5] = averageReadings[5] + gyro['z']
                        
                        time.sleep(sample_delay) # 200Hz sample rate from SMPLRT_DIV, 0x04 line
                    # set as parent loop, as it is slower. wait for the time required fo gyro/acc to stabilize
                mag = MPU9250.readMagnet(self)
                averageReadings[6] = averageReadings[6] + mag['x']
                averageReadings[7] = averageReadings[7] + mag['y']
                averageReadings[8] = averageReadings[8] + mag['z']
            
        elif readMag is False:
            #Starting at 1 and going to 40 (40 readings)
            for ACC_GYRO in range (acc_gyro_samples):
                if readAcc is True:    
                    accel = mpu9250.readAccel()
                    averageReadings[0] = averageReadings[0] + this.accel['x']
                    averageReadings[1] = averageReadings[1] + this.accel['y']
                    averageReadings[2] = averageReadings[2] + this.accel['z']

                if readGryo is True:
                    gyro = mpu9250.readGyro()
                    averageReadings[3] = averageReadings[3] + this.gyro['x']
                    averageReadings[4] = averageReadings[4] + this.gyro['y']
                    averageReadings[5] = averageReadings[5] + this.gyro['z']
                
                time.sleep(sample_delay) # 200Hz sample rate from SMPLRT_DIV, 0x04 line


        if readTemp is True:
            #Area for temperature reading if needed      
            averageReadings[9] = averageReadings[9] + this.temp 
            
            #Sleep for the period required to geet accurate reading from Acc/Gryo before incrementing the loop
            time.sleep(sample_delay)
            
        #Still inside while loop, calculate the approximately 0.5 second averages for Gyro/Acc
        #Note1: The total range is the x,y,z in averageReadings, and 20 is from outer values-reading for loop
        #Note2: The format below reads six times, from 0 to 5 (not including 6)
        for eachValue in range (0, 6):
            averageReadings[eachValue] = averageReadings[eachValue]/acc_gyro_samples

        #Still inside while loop, calculate the approximately 0.5 second averages for Mag
        #Note that the total range is the x,y,z in averageReadings, and 20 is from outer values-reading for loop
        for eachValueMag in range (6, 9):
            averageReadings[eachValueMag] = averageReadings[eachValueMag]/20    #Total samples taken from 
        
        # Calculate the magnetometer's magnitude 
        magMag = math.sqrt(averageReadings[6] ** 2 + averageReadings[7] ** 2 + averageReadings[8] ** 2)
        
        # Calculate speed from accelerometer
        speed = total_sample_time * math.sqrt(averageReadings[0] ** 2 + averageReadings[1] ** 2 + averageReadings[2] ** 2)
        
        # ToDo: Change dict key-value pair (mimics c switch statment)
        if readAcc is False and readGyro is False and readMag is False:
            return 0
        
        elif getMagnitude == False and getVelocity == False:
            # Only return the average readings and total sample time
            return averageReadings, total_sample_time
        
        elif getMagnitude == True and getVelocity == False:
            # Return the average readings, total sample time, and the magnetometer's magnitude
            return averageReadings, total_sample_time, magMag
        
        elif getMagnitude == False and getVelocity == True:
            # Return the average readings, total sample time, and the speed
            return averageReadings, total_sample_time, speed
        
        elif getMagnitude == True and getVelocity == True:
            # Return the average readings, total sample time, the magnetometer's magnitude and speed            
            return averageReadings, total_sample_time, magMag, speed
    
    # Change of Basis 
    # @param [in] self The object pointer.
    # @param [in] data1 B1
    # @param [in] data2 B2
    # @param [in] data3 d    
    # @retval Value [a]_B (int 16bit)
    def changeOfBasis(self, x1UnderB, x2UnderB, userUnderB):

        # Copy the contents ofthe vectors
        self.v1 = x1UnderB.copy()
        self.v2 = x2UnderB.copy()
        self.b = userUnderB.copy()

        # divide first row by first value in first row
        self.v2[0] = self.v2[0]/self.v1[0]
        self.b[0] = self.b[0]/self.v1[0]           # now v1[0] = 1
        
        # two vectors form the two-dimensional change of basis, first round of operation zeroes the left column
        # This block zeroes the column below the v1[0]
        # Note: Do not normalize until last step and ignore for the z-row (not included, as it does not span R3, but is in R3)
        self.v2[1] = self.v1[1] * self.v2[0] - self.v2[1]    # multiply the ratio of x and y to next vector x-value, subtract y
        self.b[1] = self.v1[1] * self.b[0] - self.b[1]       # repeat for third column

        # also divide row 1 by value in first column
        self.b[1] = self.b[1]/self.v2[1]       # now v2[1] = 1
        
        # b[0] is changed from zeroing of v2's x and z. 
        self.b[0] = self.v2[0] * self.b[1] - self.b[0]              # already correct output from block aove
        
        # return the x and y values. These are the user's coordinates on the grid
        return self.b[0], self.b[1]

    def normalizeProjection(self, xlen, ylen):
        self.length = 0.0       
        
        # normalize the values by dividing components by the length of the 
        length = math.sqrt(xlen**2 + ylen**2)
        xlen = xlen / length
        ylen = ylen / length

        return xlen, ylen

    # getAngles()
    # @param [in] self The object pointer.
    # @param [in] data1 [a_x]_B
    # @param [in] data2 [a_y]_B 
    # @retval Value [a]_B (int 16bit)
    def getPolar(self, xval, yval):
        #polar coordinates
        r = 0.0
        theta = 0.0

        r = math.sqrt(xval**2 + yval**2)    # should be one if values were normalized first
        theta = math.atan2(yval, xval)      # returned in radians from -pi to pi
        
        return r, theta


	
    # printRegData
    # prints the contents of average and instantaneous data from registers 
    # Object passed into printRegData must be a 10-value array of data from the IMU, 
    # as well as the number (int) of times the register's instantaneous data should be read.
    def printRegData(self, averageReadings, instantaneousRegRead):
        # Check averageReadings. If an array of length 10, or it is a string, take a new average reading
        if averageReadings.__len__(self) == 10 is False and isinstance(s, basestring) is False:
            # Need to store sample time, as it is returned
            averageReadings, sample_time = this.readIMU(False, False)

        # Print out the passed vaules
        print("\nAveraged Acceleration: (", averageReadings[0], ", ", averageReadings[1], ", ", averageReadings[2], ")" )
        print("\nAveraged Magnetometer Reading: (", averageReadings[6], ", ", averageReadings[7], ", ", averageReadings[8], ")" )        
        print("\nAverage Gyroscope Readings: (", averageReadings[3], ", ", averageReadings[4], ", ", averageReadings[5], ")" )
        print("\nTotal Sample Time: ", sample_time, " seconds")
                
        if instantaneousRegRead > 0:
            for readAmounts in range (instantaneousRegRead):
                print("Read # ", readAmounts)
                print("/nax = " , ( this.readAccel['x']))
                print("/nay = " , ( this.readAccel['y']))
                print("/naz = " , ( this.readAccel['z']))

                gyro = mpu9250.readGyro()
                print("/ngx = " , ( this.readGyro['x']))
                print("/ngy = " , ( this.readGyro['y']))
                print("/ngz = " , ( this.readGyro['z']))

                mag = mpu9250.readMagnet()
                print("/nmx = " , ( this.readGyro['x']))
                print("/nmy = " , ( this.readGyro['y']))
                print("/nmz = " , ( this.readGyro['z']))   
                
                # After instantaneous reads, return 
                return
        else:
            # If no instantaneous data is required, return.
            return

# This program implements the above classe to help navigate the visually impaired
while True:
   
    # ToDo: setup threads. Four cores, four threads
    # ToDo: check on initialized variables in while loops if they are global or localwhile 
    # Core 1: QR code scan
    # Core 2: Shortest Path
    # Core 3: Feedback
    # Core 4: talk to text/ text to talk
    ## setup class variables
    input = Input()
    camera = cameraGUI()
    graph = Graph()
    imu = MPU9250()
    haptic = Adafruit_DRV2605()
    feedback = Feedback()
    
    ## 'Global' variables
    ## check if these are used
 
    turn_left = False          # Used for feedack from either/ both grid results and current
    turn_right = False         #
    turn_around = False        #
    distance_update = False    #
    dist_remain = 0             # distance until destination or corne    
    
    ## These variables are used
    distance = 0.0              # used to return distance value from calls to shortest path
    dist_rem = distance         #
    path = {}                   # default dictionary
    qr_code = ""                # rerived QR code from pi cam scan
    end_node = ""               # end of path
    current_node = ""           # state == 1 requires this to start scanning from the camera
    select_dest = False         # informs the program when destination has been selected    
    button_state  = False       # used for confirming button pushes
    state = 0                   # Keeps track of the current state the program exists in 
    destination = 0             # a room number
    counter = 0               # generic counter used in all states for feedback after specific iterations through a loop
    max_counter = 1000000              # maximum count @Param:counter reaches
    
    ## Magnetometer Variables
    getMag = True
    getAcc = True
    getGyro = True
    magnitudeMagnetometer = True
    speed = True

    noMag = False
    noAcc = False
    noGyro = False
    noMagMag = False
    noSpeed = False
    
    ave_sample_time = 0.0                                        # sample time of the last reading from the IMU
    ave_readings = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]    # stores last readings of IMU
    A_x = [-3.5191, 32.6908, 55.998]                    # grid's mag reading in the +x-direction
    A_y = [-5.1537, 8.24405, 55.108]                    # grid's mag reading in the +y-direction
    ax_under_b = 0.0
    ay_under_b = 0.0




    # used for labeling nodes
    for node in ['A', 'B', 'C', 'D', 'E', 'F', 'G','H', 'I', 'J', 'K',
                 'L', 'M','N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V']:

        graph.add_node(node)

    # used for selecting destination
    graph.rooms = OrderedDict(sorted({'A':[340, 341, 342, 343], 'B':[344,345,346,347], 'C':[348,349,350,351], 
                'D':[353], 'E':[355], 'F':[357], 'G':[359],'H':[361], 'I':[363], 'J':[365], 'K':[367],
                'L':[369], 'M':[371],'N':[373], 'O':[375], 'P':[377], 'Q':[379], 'R':[381], 'S':[383], 'T':[385], 'U':[387], 'V':[389]}.items()))



    graph.add_edge('T', 'A', -65, 0)
    graph.add_edge('A', 'B', 0, 21)
    graph.add_edge('B', 'C', 0, 28)  #30 across, going up.
    graph.add_edge('C', 'D', 0, 112)
    graph.add_edge('D', 'E', 0, 40)

    graph.add_edge('A', 'T', 65, 0)
    graph.add_edge('B', 'A', 0, -21)
    graph.add_edge('C', 'B', 0, -28) # going down
    graph.add_edge('D', 'C', 0, -112)
    graph.add_edge('E', 'D', 0, -40)

# room E to J
    graph.add_edge('E', 'F', 39, 0)
    graph.add_edge('F', 'G', 34, 0)
    graph.add_edge('G', 'H', 27, 0)  #30 across, ->
    graph.add_edge('H', 'I', 45, 0) 
    graph.add_edge('I', 'J', 38, 0)

    graph.add_edge('F', 'E', -39, 0)
    graph.add_edge('G', 'F', -34, 0)
    graph.add_edge('H', 'G', -27, 0)  #<--Direction
    graph.add_edge('I', 'H', -45, 0)
    graph.add_edge('J', 'I', -38, 0)

    # rooms J to P

    graph.add_edge('J', 'K', 12, 0)
    graph.add_edge('K', 'L', 5, 0)
    graph.add_edge('L', 'M', 5, 0)
    graph.add_edge('M', 'N', 4, 0)  #30 across, direction going down
    graph.add_edge('N', 'O', 4, 0)

    graph.add_edge('K', 'J', -12, 0)
    graph.add_edge('L', 'K', -5, 0)
    graph.add_edge('M', 'L', -5, 0)
    graph.add_edge('N', 'M', -4, 0)  #30 across, direction going UP
    graph.add_edge('O', 'N', -4, 0)

    #rooms O to V

    graph.add_edge('O', 'P', 8, 0)
    graph.add_edge('P', 'Q', 7, 0)
    graph.add_edge('Q', 'R', 5, 0)  #30 ACROSS  <<--
    graph.add_edge('R', 'S', 4, 0)
    graph.add_edge('S', 'T', 6, 0)

    graph.add_edge('P', 'O', -8, 0)
    graph.add_edge('Q', 'P', -7, 0)
    graph.add_edge('R', 'Q', -5, 0)  #30 ACROSS  ->>
    graph.add_edge('S', 'R', -4, 0)
    graph.add_edge('T', 'S', -6, 0)

    
    # start at an arbitrary room in the nodes dictionary stored in graph
    destination = 340
    feedback.update_room(destination)

    state = 1

    # This state scans for initial input for a reference point on the grid
    while state  == 1:
        # reset select_dest
        select_dest = False
        
        # ensure button timer (used in state == 2) is reset
        btn_timer = 0

        # Note that graph has end_node saved as the local variable end
        end_node = graph.get_dest(destination)
        feedback.update_room(destination)
        
        # notify the user with tts instructions
        os.system(feedback.select_room_tts)
        os.system(feedback.current_dest_tts)
        
        ## READ FOR DESTINATION SELECTION
        while select_dest == False:
            if counter > max_counter:
                counter = 0
                os.system(feedback.current_dest_tts)
                os.system(feedback.select_room_tts)
            
            
            # scan push button pins for user-input.
            if GPIO.input(select_next):
                if input.debounce(select_next) == True:
                    destination = graph.next_room(end_node, destination)
                    feedback.update_room(destination)
                    os.system(feedback.room_change_tts)
                    end_node = graph.get_dest(destination)
                    feedback.room = destination
                    counter = 0
                    
            #   call text to talk function, passing the value of current node
            if GPIO.input(select_prev):
                # debounce
                if input.debounce(select_prev) == True:
                    destination = graph.prev_room(end_node, destination)
                    feedback.update_room(destination)
                    os.system(feedback.room_change_tts)
                    end_node = graph.get_dest(feedback.room)
                    feedback.room = destination
                    counter = 0
                    
            #   call text to talk, passing current_node 
            if GPIO.input(select_current):   
                # confirm button hit, debouce
                if input.debounce(select_current) == True:
                    # Get confirmation
                    os.system(feedback.get_confirm_dest_tts)
                    confirm = True
                    while confirm == True:
                        # if too much time has elapsed without user input, remind them of the activity
                        if counter > max_counter:
                            # reset counter
                            counter = 0
                            #notify user of the current activity
                            os.system(feedback.get_confirm_dest_tts)

                        # selection confirmed, now debounce
                        if GPIO.input(select_current):   
                            # confirm button hit, debouce
                            if input.debounce(select_current) == True:    
                                # notify user of selection    
                                ## ToDo: change this so that when users re-enter state == 1 
                                ## (i.e. current_node is not ""), that the current position is 
                                ## loaded
                                os.system(feedback.confirm_dest_tts)                
                                
                                # Save the last node for comparing scanned qr_codes
                                # Note that graph has end_node saved as the local variable end
                                end_node = graph.get_dest(destination)
                                feedback.update_room(destination)
                                
                                # exit while loop
                                confirm = False
                                select_dest = True
                                #reset counter
                                counter = 0
                                # go to the next state if the program has progressed outside of
                                # the inner while loop
                                state = 2 
                        
                        # User wants to change selection
                        if GPIO.input(select_next):
                            # debounce
                            if input.debounce(select_next) == True:
                                # notify user of new activity                                   
                                os.system(feedback.select_room_tts)
                                confirm = False
                                counter = 0
                                
                        # User wants to change selection
                        if GPIO.input(select_prev):
                            # debounce
                            if input.debounce(select_prev) == True:
                                # notify user of new activity
                                os.system(feedback.select_room_tts)
                                confirm = False
                                counter = 0
                                
                        # no selection has been made, so increase timer
                        counter += 1
                        
            # no selection has been made, so increase timer
            counter += 1
                        
        
    # state ==  2
    # get start node
    # return to state == 1 if:
    #   qr_code == destination
    # proceed to state == 3 if:
    #   qr_code is in graph.node and not destination
    while state == 2:
        print("in state == 2")
        # scan for a QR code
        qr_code = camera.qrGet()                  # camera.qrGet() returns a string
        # if this qr_code has a value, a qr_code was scanned
        # also check if the qr_code belongs to a value on the grid
        if (camera.resultQR is not "") and (camera.resultQR in graph.rooms.keys()):
            
            # expected values
            if qr_code is not end_node:
                
                #might need to check if the qr_code is the same as destination, but I think that shortest_path already does this
                distance, feedback.new_path = shortest_path(graph, qr_code, destination)
                
                feedback.prev_path = feedback.new_path  # only do this without processing the new_path first in state 2

                # get the starting node and save it to feedback.room in order to
                feedback.room = graph.get_room(new_path[0])
                
                # tell the user where they are starting
                os.system(feedback.start_node)
                
                # get user's orientation to the grid from magnetometer
                ave_readings, ave_sample_time = imu.readIMU(noAcc, noGyro, getMag, noMagMag, noSpeed)
                
                # set user in correct direction using readings and image matrix
                # steps done below: change basis projection, normalize result, and set Feedback.self to some direction
                feedback.processOrientation(imu.normalizeProjection(imu.changeOfBasis(A_x, A_y, ave_readings[6:9])))
                
                # provide tts feedback
                if feedback.turn == feedback.forward:
                    os.system(feedback.forward_tts)
                else:
                    feedback.process_turn()
                    
                # followed by haptic feedback
                feedback.direct_tts()
                
                # State changes only when qr is different from destination is state equal to 2
                state = 3
                
            # next check if qr_code is the destination
            elif camera.resultQR is end_node:
                
                # make call to ask for user-input destination, as scanned module is already the current module
                # ToDo: make call to text to talk (user has arrived at destination)
                os.system(feedback.location_found_tts)
                os.system(feedback.at_destination_tts)
                state = 1
                
            # default is an unexpected value, so break and it will re-scan for qr code
            else:
                
                # keep scanning for QR codes
                qr_code = ""

        # user somehow scanned a junk QR code.
        else:
            qr_code = ""
            
        # check if user manually interrupts to change destination
        if GPIO.input(select_next) or GPIO.input(select_prev) or GPIO.input(select_current):
            
            # read all input pins "efficiently"            
            # user has pushed a button
            if input.deb_mult([select_next, select_prev, select_current]) == True:
                # ToDo: get confirmation on destiantion change request

                # tell user what's up
                os.system(feedback.confrim_change_dest_tts)
                
                # allow the user to reselect input
                state = 1

    # state == 3
    # this state is actively providing feedback while scanning for new input
    #  feedbaack consists of reading IMU
    # providing haptic feedback
    # once destination == current_node (origin), got to state 3
    while state == 3:
        # scan for a QR code
        camera.qrGet()                  # camera.qrGet() returns a string

        # if this qr_code has a value, a qr_code was scanned
        # also check if the qr_code belongs to a value on the grid
        if (camera.resultQR is not "") and (camera.resultQR in graph.node.keys()):
            
            # create a new path using the scanned QR-code
            destination, feedback.new_path = shortest_path(graph, qr_code, destination)
            
            # find the direction to turn or the next command
            feedback.direct()
            feedback.process_turn()
            feedback.direct_tts()
            haptic.process_turn(feedback.effect)
            
            # if the end has been reached, go back to state == 1
            if feedback.check_turn is True:

                # make call to ask for user-input destination, as scanned module is already the current module
                state = 1
                
            # otherwise, continue on the path, making sure to reset qr_code
            else:
                
                # save new_path
                feedback.prev_path = feedback.new_path
                
                # keep scanning for QR codes
                qr_code = ""

        # user somehow scanned a junk QR code.
        else:
            camera.resultQR = ""
            
        # check if user manually interrupts to change destination
        if GPIO.input(select_next) or GPIO.input(select_prev) or GPIO.input(select_current):
            
            # read all input pins "efficiently"            
            # user has pushed a button
            if input.deb_mult([select_next, select_prev, select_current]) == True:
                # ToDo: get confirmation on destiantion change request

                # tell user what's up
                os.system(feedback.confrim_change_dest_tts)               
                # allow the usere to reselect input
                state = 1


 
