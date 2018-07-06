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
GPIO.setup(select_next, GPIO.IN, )
GPIO.setup(select_prev, GPIO.IN)
GPIO.setup(select_current, GPIO.IN)


# Class used for processing input, such as debounce
class Input():
    
    def __init__(self):
        self.max_count = 1000


    def debounce(self, gpio_pin):
        counter = 0
        button_state = False
        
        for count_on in range(1000):
            if GPIO.input(gpio_pin):
                counter += 1

        if counter >= self.max_count and GPIO.input(gpio_pin):
            button_state = True

        return button_state
    
    def deb_mult(self, pins):
        counter = 0
        button_state = False
        button = 0
        
        for count_on in range(1000):
            if len(pins) > 0:
                for pin in range(len(pins)):
                    if GPIO.input(pins[pin]):
                        button = pins[pin]
                        counter += 1
          #  else:
                # oops! what to do?
  
                    

        if counter >= self.max_count and GPIO.input(button):
            button_state = True

        return button_state


class Graph(object):
    def __init__(self):
        self.prev_node = 0
        self.nodes = set()
        self.rooms = OrderedDict()
        self.edges = defaultdict(list)
        self.Xdistances = {}    # fast initialization of an empty dict
        self.Ydistances = {}    # fast initialization of an empty dict
        self.vertices_2D = {}   # empty tuple
        self.node_cart_coord = {} # fast initialization of an empty dict

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

##        # as long as rooms are ints, this local variable will work
##        last_value = 0
##        first_value = 0
##
##        # Used to denote when the last entry in the dictionary is the current_room, 
##        first_entry = False
##        # local variable used for checks if the next key in ordered dict is necessary
##        next_entry = False
##
##
##        
##        # get last dictionary value in dictionary
##        for node, room in self.rooms.items():
##            prev_node = node
##
##        # get first and last dictionary value in dictionary
##        first_value = self.rooms[next(iter(self.rooms))][0]
##        last_value = self.rooms[node][len(self.rooms[node]) - 1]
##
##        # Initialize prev_node to key 
##        prev_node = key
##
##        # If the first value is the current room, return the last value in the list
##        if first_value == current_room:
##
##            # return the last value in the dictionary
##            return last_value
##
##        # If this is not the case, then find the location in the dictionary of the previous room
##        else:
##            
##            # get nodes dictionary and find the current room
##            for node, room in self.rooms.items():
##                
##                # found the correct key 
##                if key == node:
##                    
##                    # loop through set at dictionary entries that have another value
##                    if len(room) > 1:
##
##                        # check the list at key "key" in Graph's rooms orderedDict for the previous room
##                        for index in range(len(room)):
##
##                            # if the value exists, return the next value in the array
##                            if current_room is room[index] and index > 0:
##                                
##                                # prev room is the next room in the set at dictionary entry node
##                                return room[index - 1]
##
##                    # There is only one element, so return the previus node's value
##                    else:
##
##                        # Return the last element in the last key's list
##                        return self.rooms[prev_node][len(room) - 1]
##
##                # As the search for the previouse node is saught after in this function, save update the previous node
##                prev_node = node
                    





        
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
                self.prev_node = node
            # otherwise, node has equaled key, so return the last value of the previous key
            elif prev_entry == True and self.rooms[self.prev_node][len(self.rooms[self.prev_node])-1]:
                # return the last entry in the previous list at the key prev_node
                return self.rooms[self.prev_node][len(self.rooms[self.prev_node])-1]

        # outside while loop, if nothing else returned, pass last value of last node
        return self.rooms[self.prev_node][len(self.rooms[self.prev_node])-1]


class Feedback():   
    
    def __init__(self):
        self.origin = {}    # where we start
        self.graph = {}         # 
        self.direction = {}     # empty dictionary
        self.path = set()  # get the path letters (it should be a list)
        self.new_path = []
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
        self.arrived = "flite -t 'You have arrived at room " + str(self.room) + ".'"
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
        self.update_current_loc_tts = "flite -t 'You are near room " + str(self.room) + ".'"
        self.confrim_change_dest_tts = 'flite -t "Button push detected." '


    def update_room(self, room):
        self.arrived_tts = "flite -t 'You have arrived at room " + str(room) + ".'"
        self.current_dest_tts = "flite -t 'Your current destination is room " + str(room) + ".'"
        self.get_confirm_dest_tts = "flite -t 'The selected destination is " + str(room) + ". Please confirm.'"
        self.confirm_dest_tts = "flite -t 'You have selected room " + str(room) + ". Scanning for current position.'"
        self.start_node = "flite -t 'You are currently standing by room " + str(room) + ". Plotting course.'"
        self.room_change_tts = "flite -t 'Room " + str(room) + ".'"
        self.update_current_loc_tts = "flite -t 'You are near room " + str(room) + ".'"
    
        
    # direct(self)
    # compares two lists
    # returns direction in for of string
    def direct(self):
        # initialize variables each time.
        # if they were overwritten outside the function call to turn, it will not alter the behavior
        self.displacement = [0,0]
        self.ThreeTwoDisp = [0,0]
        self.TwoOneDisp = [0,0]
        self.path_index = 0
        self.poi_start = self.new_path[0]
        self.wrong_way = False
        dispx = 0
        dispy = 0
        path_length = len(self.prev_path)
        
        
        # Check if destination has been reached
        if self.poi_start is self.new_path[len(self.new_path)-1]:

            # User has reached their destination
            self.turn = self.destination
            return


        #   Expecting:
        #       poi_start to be a node
        #       prev_path to be a vector of nodes
        elif self.poi_start not in self.prev_path:
            
            # you have gone the wrong way
            self.turn = self.turn_around

            # MAY NOT BE REQUIRED
            self.wrong_way = True

        # If new_path only has one element, user has reached the destination
        elif len(self.new_path) is 1:
              
            # CHECK IF THIS IS THE CASE
            self.turn = self.destination
            
                        
        # Otherwise, if the start point exists in the previous path, run some checks
        elif self.poi_start in self.prev_path:

            # Checks:
            #       length of previous path,
            #       compares length of prev_path and new_path
            if path_length >= 2:

                if self.start_node is not self.prev_path[0]:
                    # Expecting a node, specifically the next node in the path after poi_start node
                    self.poi_prev = self.prev_path[self.prev_path.index(self.poi_start)-1]

                    # case where the node exists on the previous path, and the previoius path has at least 3 nodes
                    if path_length > 2:
                        
                        # Expecting a node, specifically the next node in the path after poi_next node
                        self.poi_last = self.prev_path[self.prev_path.index(self.poi_start)+1]

                        # This if else block check all left/right combo options
                        # Note: first four checks are for clockwise rotation,
                        #       next four checks are for counter-clockwise rotations.

                        # start is above prev and last is right of start
                        if self.graph[self.poi_prev][1] < self.graph[self.poi_start][1] and self.graph[self.poi_last][0] > self.graph[self.poi_start][0]:
                            self.turn = str(self.right).strip("\n")
                            
                        # start is right of prev and last is below start
                        elif self.graph[self.poi_prev][0] < self.graph[self.poi_start][0] and self.graph[self.poi_last][1] < self.graph[self.poi_start][1]:
                            self.turn = str(self.right).strip("\n")

                        # start is below prev and last is left of start
                        elif self.graph[self.poi_prev][1] > self.graph[self.poi_start][1] and self.graph[self.poi_last][0] < self.graph[self.poi_start][0]:
                            self.turn = str(self.right).strip("\n")
                            
                        # start is left of prev and last is above start
                        elif self.graph[self.poi_prev][0] > self.graph[self.poi_start][0] and self.graph[self.poi_last][0] < self.graph[self.poi_start][0]:
                            self.turn = str(self.right).strip("\n")

                        # start is right of prev and last is above start
                        elif self.graph[self.poi_prev][0] < self.graph[self.poi_start][0] and self.graph[self.poi_last][1] > self.graph[self.poi_start][1]:
                            self.turn = str(self.left).strip("\n")

                        # start is above prev and last is left of start
                        elif self.graph[self.poi_prev][1] < self.graph[self.poi_start][1] and self.graph[self.poi_last][0] < self.graph[self.poi_start][0]:
                            self.turn = str(self.left).strip("\n")

                        # start is left of prev and last is below start
                        elif self.graph[self.poi_prev][0] > self.graph[self.poi_start][0] and self.graph[self.poi_last][1] < self.graph[self.poi_start][1]:
                            self.turn = str(self.left).strip("\n")

                        # start is below prev and last is right of start
                        elif self.graph[self.poi_prev][1] > self.graph[self.poi_start][1] and self.graph[self.poi_last][0] > self.graph[self.poi_start][0]:
                            self.turn = str(self.left).strip("\n")

                        # default turn instruction
                        else:
                            self.turn = self.forward

                        
                        # make sure to preserve the prev_path if the new_path is length 2
                        if len(self.new_path) > 2:

                            # updates path
                            self.prev_path = self.new_path



        return


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


graph = Graph()
input = Input()
feedback = Feedback()

# used for labeling nodes
for node in ['A', 'B', 'C', 'D', 'E', 'F', 'G','H', 'I', 'J', 'K',
             'L', 'M','N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V']:

    graph.add_node(node)

# used for selecting destination
graph.rooms = OrderedDict(sorted({'A':[340, 341, 342, 343], 'B':[344,345,346,347], 'C':[348,349,350,351], 
            'D':[353], 'E':[355], 'F':[357], 'G':[359],'H':[361], 'I':[363], 'J':[365], 'K':[367],
            'L':[369], 'M':[371],'N':[373], 'O':[375], 'P':[377], 'Q':[379], 'R':[381], 'S':[383], 'T':[385]}.items()))

graph.node_cart_coord = OrderedDict(sorted({'A':[0,0], 'B':[0,21], 'C':[0,49], 'D':[0,161], 'E':[0,201], 
            'F':[39,201], 'G':[73,201],'H':[100,201], 'I':[145,201], 'J':[183,201],
            'K':[183,161], 'L':[183,100], 'M':[183,49],'N':[183,21], 'O':[183,0],
            'P':[163,0], 'Q':[133,0], 'R':[100,0], 'S':[80,0], 'T':[65,0]}.items()))




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

graph.add_edge('J', 'K', 0, -40)
graph.add_edge('K', 'L', 0, -61)
graph.add_edge('L', 'M', 0, -51)
graph.add_edge('M', 'N', 0, -28)  #30 across, direction going down
graph.add_edge('N', 'O', 0, -21)

graph.add_edge('K', 'J', 0, 40)
graph.add_edge('L', 'K', 0, 61)
graph.add_edge('M', 'L', 0, 51)
graph.add_edge('N', 'M', 0, 28)  #30 across, direction going UP
graph.add_edge('O', 'N', 0, 21)

#rooms O to t

graph.add_edge('O', 'P', -20, 0)
graph.add_edge('P', 'Q', -30, 0)
graph.add_edge('Q', 'R', -33, 0)  #30 ACROSS  <<--
graph.add_edge('R', 'S', -20, 0)
graph.add_edge('S', 'T', -15, 0)

graph.add_edge('P', 'O', 20, 0)
graph.add_edge('Q', 'P', 30, 0)
graph.add_edge('R', 'Q', 33, 0)  #30 ACROSS  ->>
graph.add_edge('S', 'R', 20, 0)
graph.add_edge('T', 'S', 15, 0)

destination = 340

while True:

        # reset select_dest
        select_dest = False

        # Note that graph has end_node saved as the local variable end
        end_node = graph.get_dest(destination)
        feedback.update_room(destination)
        
        ## READ FOR DESTINATION SELECTION
        while select_dest == False:
            
            
            # scan push button pins for user-input.
            if GPIO.input(select_next):
                if input.debounce(select_next) == True:
                    destination = graph.next_room(end_node, destination)
                    feedback.update_room(destination)
                    end_node = graph.get_dest(destination)
                    feedback.room = destination
                    print("Destination: ", destination)
                    
            #   call text to talk function, passing the value of current node
            if GPIO.input(select_prev):
                # debounce
                if input.debounce(select_prev) == True:
                    destination = graph.prev_room(end_node, destination)
                    feedback.update_room(destination)
                    end_node = graph.get_dest(destination)
                    feedback.room = destination
                    print("Destination: ", destination)
                    print("destination node: ", end_node)
                    
            #   call text to talk, passing current_node 
            if GPIO.input(select_current):   
                # confirm button hit, debouce
                if input.debounce(select_current) == True:
                    # Get confirmation
                    os.system(feedback.get_confirm_dest_tts)
                    confirm = True
                    while confirm == True:


                        # selection confirmed, now debounce
                        if GPIO.input(select_current):   
                            # confirm button hit, debouce
                            if input.debounce(select_current) == True:          
                                
                                # Save the last node for comparing scanned qr_codes
                                # Note that graph has end_node saved as the local variable end
                                end_node = graph.get_dest(destination)
                                feedback.update_room(destination)
                                
                                # exit while loop
                                confirm = False
                                select_dest = True
                                print("Destination: ", destination)
                        
                        # User wants to change selection
                        if GPIO.input(select_next):
                            # debounce
                            if input.debounce(select_next) == True:
                                confirm = False
                                counter = 0
                                
                        # User wants to change selection
                        if GPIO.input(select_prev):
                            # debounce
                            if input.debounce(select_prev) == True:
                                confirm = False
                                counter = 0
