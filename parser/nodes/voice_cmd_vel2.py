#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String


# Control word to indicate the beginning of a destination command.
request_start = "find";

# Control word to indicate the end of a destination command.
request_end = "please";

# Control word to abort a command in progress.
request_cancel = "sorry";

# Control word to pause navigation.
drive_pause = "wait";

# Control word to resume navigation.
drive_resume = "continue";

command_words = [request_start,request_end,request_cancel,drive_pause,\
                 drive_resume];
                 
one_digit_words = ["zero","one","two","three","four","five","six","seven",\
                   "eight","nine","oh"];
                   # value = index % 10;
                
two_digit_words = ["ten","eleven","twelve","thirteen","fourteen","fifteen",\
                   "sixteen","seventeen","eighteen","nineteen"];
                   # value = index + 10;
                   
tens_digit_words = ["twenty","thirty","forty","fifty","sixty","seventy",\
                    "eighty","ninety"];
                    # value = 10 * index + 20;

number_words = one_digit_words + two_digit_words + tens_digit_words
                
male_words = ["men's","men","boy's","boys","boy"];
                
female_words = ["women's","women","ladies","lady","girl's","girls","girl"];

gender_words = male_words + female_words
                
restroom_words = ["restroom","bathroom","rest","bath"];
                
other_dest_words = ["exit","elevator"];
                    
direction_words = ["north","south"];

office = {"Adali": 324, "Banerjee": 362, "Bargteil": 341, "Carter": 308, \
          "Chen": 357, "Choa": 303, "Desjardins": 337, "Finin": 329, \
          "Halem": 330, "Kim": 312, "Kalpakis": 348, "Laberge": 358, \
          "Lomonaco": 306, "Marron": 359, "Matuszek": 331, "Menyuk": 304, \
          "Mohsenin": 323, "Morris": 308, "Nicholas": 356, "Oates": 336, \
          "Olano": 354, "Patel": 322, "Pearce": 373, "Peng": 327, \
          "Phatak": 319, "Pinkston": 327, "Pirsiavash": 342, "Rheingans": 355, \
          "Robucci": 316, "Schmill": 350, "Sidhu": 347, "Slaughter": 311, \
          "Syed": 301, "Weiss": 302, "Yan": 315, "Younis": 318, "Zhu": 360};
          
# UNCOMMENT THE FOLLOWING LINE TO IGNORE PROFESSOR NAMES
#office = {};

professors = office.keys()
                
dest_words = number_words + gender_words + restroom_words + other_dest_words + \
             direction_words + professors + ["room"];
                             
# Master list of all words to listen for.
watch_words = command_words + dest_words

# Int values of number words.
value_of = {"zero": 0, "oh": 0, "one": 1, "two": 2, "three": 3, "four": 4, \
            "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, \
            "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, \
            "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18, \
            "nineteen": 19, "twenty": 20, "thirty": 30, "forty": 40, \
            "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90};

# Storage space for latest value from parser
new_string = "";
# Indicates whether robot is currently driving
nav_mode = False
# Indicates whether robot currently has a destination.
has_dest = False

# Storage space for counts of words in a command that match a certain subset.
male_count = 0;
female_count = 0;
odest_count = 0;
restroom_count = 0;

# Represents the most recent direction mentioned in the command. (1:N, 2:S)
direction = 0;



class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('husky_nav', String)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
'''
        if msg.data.find("full speed") > -1:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
        if msg.data.find("half speed") > -1:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2

        if msg.data.find("forward") > -1:    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif msg.data.find("left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:        
                self.msg.angular.z = self.speed*2
        elif msg.data.find("right") > -1:    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:        
                self.msg.angular.z = -self.speed*2
        elif msg.data.find("back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:          
            self.msg = Twist()
        '''
        new_string = msg.data;
        if (nav_mode):
            # nav_mode: state is DRIVING (if has_dest) or RUNAWAY (if !has_dest).
            # Behavior: pause if drive_pause or RUNAWAY; otherwise no change.
            if (new_string == drive_pause || !has_dest):
                # tell navigation to stop without discarding destination
                # ...
                
                # switch to PAUSED
                nav_mode = False
        else if (has_dest && new_string == drive_resume):
            # !nav_mode && has_dest: PAUSED.
            # new_string == drive_resumeresume: received resume command.
            # Behavior: resume driving, switch to nav_mode.
            
            # pass resume command to navigation stack
            # ...
            nav_mode = True;
        else:
            # !(nav_mode): PAUSED (if has_dest) or READY (if !has_dest).
            # !(has_dest && new_string == drive_resume): if PAUSED, didn't receive
            # resume command.
            # Behavior: If request_start was received, parse destination command.
            #    If request_start wasn't received and !has_dest, prompt user for
            #    destination command. Otherwise, no change.
            
            if (new_string == request_start):
                # Receiving new destination command. Store each word in sequence
                # until command is complete or canceled.
                words = [];
                numbers_used = [];
                prof_name = "";
                direction = 0;
                male_count = 0;
                female_count = 0;
                odest_count = 0;
                restroom_count = 0;
                while (new_string != request_cancel && new_string != request_end):
                    # load value to new_string from pocketsphinx
                    # ...
                    
                    # If new_string contains a word, add it to the sequence.
                    # Note which category/ies it belongs to.
                    if (new_string != ""):
                        words.append(new_string);
                        if (new_string == "north"):
                            direction = 1;
                        else if (new_string == "south"):
                            direction = 2;
                        if (new_string in male_words):
                            male_count++;
                        if (new_string in female_words):
                            female_count++;
                        if (new_string in number_words):
                            numbers_used.append(new_string)
                        if (new_string in other_dest_words):
                            odest_count++;
                        if (new_string in restroom_words):
                            restroom_count++;
                        if (new_string in professors):
                            prof_name = new_string;
                        
                        # gender_count indicates whether the destination has a specified gender (for restrooms)
                        # "gender_count > 0" is true only if one gender has been mentioned more often than the other.
                        if male_count > female_count:
                            gender_count = male_count - female_count;
                        else if female_count > male_count:
                            gender_count = female_count - male_count;
                        else:
                            gender_count = 0;
                        
                        if (new_string == "room" && gender_count > 0):
                            # "[gender] room"
                            restroom_count++;
                        
                # If command was canceled, discard sequence.
                if (new_string != request_cancel):
                    # Parse sequence.
                    if ("exit" in words):
                        # Only one possible destination.
                        # Send "exit" identifier to navigation.
                        # ...
                        self.pub_.publish("exit")
                        has_dest = True;
                    else if ("elevator" in words):
                        # Two possible destinations.
                        if (direction == 0):
                            # Send "nearest elevator" identifier to navigation.
                            # ...
                            self.pub_.publish("elevator")
                            has_dest = True;
                        else if (direction == 1):
                            # Send "north elevator" identifier to navigation.
                            # ...
                            self.pub_.publish("north elevator")
                            has_dest = True;
                        else if (direction == 2):
                            # Send "south elevator" identifier to navigation.
                            # ...
                            self.pub_.publish("south elevator")
                            has_dest = True;
                    else if (restroom_count > 0):
                        # User mentioned the restroom.
                        # Four possible destinations, plus not-fully-specified
                        # options.
                        if (female_count == 0 && male_count == 0 && direction == 0):
                            # Send "nearest unspecified restroom" to navigation.
                            # ...
                            self.pub_.publish("restroom")
                            has_dest = True;
                        else if (gender_count == 0):
                            # !(gender_count == 0 && direction_count == 0) &&
                            # (gender_count == 0).
                            # Therefore !(direction_count == 0).
                            # Get direction and send "<direction> unspecified
                            # restroom" to navigation.
                            if (direction == 1):
                                # Send "north unspecified restroom" to navigation.
                                # ...
                                self.pub_.publish("north restroom")
                                has_dest = True;
                            else:
                                # Send "south unspecified restroom" to navigation.
                                # ...
                                self.pub_.publish("south restroom")
                                has_dest = True;
                        else:
                            # gender_count != 0, direction == ?
                            # Get gender...
                            if male_count > female_count:
                                # Gender: male. Check direction...
                                if (direction == 1):
                                    # Send "north male restroom" to navigation.
                                    # ...
                                    has_dest = True;
                                else if (direction == 2):
                                    # Send "south male restroom" to navigation.
                                    # ...
                                    has_dest = True;
                                else:
                                    # Send "unspecified male restroom" to
                                    # navigation.
                                    # ...
                                    has_dest = True;
                            else:
                                # Gender: female. Check direction...
                                if (direction == 1):
                                    # Send "north female restroom" to navigation.
                                    # ...
                                    has_dest = True;
                                else if (direction == 2):
                                    # Send "south female restroom" to navigation.
                                    # ...
                                    has_dest = True;
                                else:
                                    # Send "unspecified female restroom" to
                                    # navigation.
                                    # ...
                                    has_dest = True;
                    else if (prof_name != ""):
                        # User mentioned a professor's name.
                        room_number = office[prof_name];
                        # Pass room_number to navigation.
                        # ...
                        self.pub_.publish(room_number)
                        has_dest = True;
                    else if (len(numbers_used) > 0):
                        # User mentioned a numbered room.
                        result = 0;
                        last_ten = 0;
                        for (i = 0; i < len(numbers_used); i++):
                            curr_num = numbers_used[i];
                            curr_val = value_of[numbers_used[i]];
                            if (curr_num in one_digit_words):
                                # e.g., "... three ..."
                                result *= 10;
                                result += curr_val;
                                last_ten = 0;
                            else if (curr_num in two_digit_words):
                                # e.g., "... twelve ..."
                                result *= 100;
                                result += curr_val;
                                last_ten = 0;
                            else if (curr_num in tens_digit_words):
                                # e.g., "... forty ..."
                                if (last_ten):
                                    # e.g., "... [twenty] forty ..."
                                    result *= 100;
                                else:
                                    # e.g., "... [one] forty ..."
                                    result *= 10;
                                
                                result += (curr_val/10);
                                last_ten = 1;
                        
                        if (last_ten):
                            # e.g., "... forty"
                            result *= 10;
                        
                        # Check result against list of existing room numbers.
                        # ...
                        if (result >= 300 && result <= 377):
                            # Pass result to navigation.
                            # ...
                            self.pub_.publish(result)
                            has_dest = True;
                        else:
                            pass
                            # Prompt user for new command.
                            # ...
                        
                    # Done parsing.
                    if (has_dest):
                        nav_mode = True;

                    else:
                        pass
                        # Prompt user to try rephrasing.
                        # ...
                    
            else if (!has_dest):
                pass
        







# The seeing eye robot has three main states that determine which words it's
# listening for.
# READY: Robot has not received a destination, or has arrived at the last
#        destination given.
#     (!has_dest && nav_mode)
#     Listening for new destination command (of the form "request_start,
#         [destination description], request_end").

# DRIVING: Robot has received a destination and is in motion (or planning its
#          navigation).
#     (has_dest && nav_mode)
#     Listening for drive_pause command.

# PAUSED: Robot has received a destination, but motion has been interrupted by
#         the drive_pause command.
#     (has_dest && !nav_mode)
#     Listening for drive_resume command, or for new destination command.

# (!has_dest && nav_mode) would mean the robot is driving with no set
# destination.
# This state should never arise. I've noted it as RUNAWAY.

            # ...        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

