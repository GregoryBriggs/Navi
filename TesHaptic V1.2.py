import smbus as smbus
import time
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


    def selectLibrary(self, lib):
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_LIBRARY, lib)


    def go(self):
        bus.write_byte_data(DRV2605_ADDR, DRV2605_REG_GO, 1)


    def stop(self):
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
        print("inside playEffect")        
        for packet in range (len(effect)):
            # set the effect to play
            Adafruit_DRV2605.setWaveform(packet, effect[packet])# play effect
        
        # play the effect!
        self.go()
    
#
''' End of Haptic Motor Driver Addresses '''



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


drv = Adafruit_DRV2605()
feedback = Feedback()
feedback.effect = [14, 14, 14, 14, 0]
drv.selectLibrary(DRV2605_REG_LIBRARY)
while True:
  
    print(feedback.effect)
    drv.playEffect(feedback.effect)
    time.sleep(.5)
