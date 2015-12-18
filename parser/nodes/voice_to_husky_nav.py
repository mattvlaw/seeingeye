#!/usr/bin/env python

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import Thread

from std_msgs.msg import String, Int16

# STANDING ISSUES
# Don't know how to talk to the user, e.g. if a command is invalid ("# ???")

# LIST OF OUTPUT VALUES
# Opcodes:
#   Stop & discard planned route: 0
#   Pause (stop moving but remember current route): 1
#   Resume stored route: 2 
# Numbered rooms:
#   Rooms 301-324: 301-324
#   "Room 325" (administrative office): 325
#   Rooms 326-377: 326-377
# Building exit: 100
# Elevators:
#   Nearest elevator: 110
#   South elevator: 111
#   North elevator: 112
# Restrooms:
#   Nearest restroom, no gender specified: 120
#       Nearest restroom, women: 121
#       Nearest restroom, men: 122
#   South, no gender: 123
#       South, women: 124
#       South, men: 125
#   North, no gender: 126
#       North, women: 127
#       North, men: 128

# Note on decoding restroom values:
# code = raw_value - 120
# gender = code % 3
  # 0= unspecified, 1= female, 2= male
# direction = (code - gender)/3
  # 0= unspecified, 1= south, 2= north
    
# For decoding elevator values:
# direction = raw_value - 110
  # 0= unspecified, 1= south, 2= north
    
# IMPORTANT WORDS
# Control word to indicate the beginning of a destination command.
request_start = "find"
# Control word to indicate the end of a destination command.
request_end = "please"
# Control word to abort a command in progress.
request_cancel = "sorry"
# Control word to pause navigation.
drive_pause = "wait"
# Control word to resume navigation.
drive_resume = "continue"
command_words = [request_start,request_end,request_cancel,drive_pause,\
                 drive_resume]
                 
# Number words that specify one digit of a value.
one_digit_words = ["zero","one","two","three","four","five","six","seven",\
                   "eight","nine","oh"]
# Number words that specify two digits of a value.
two_digit_words = ["ten","eleven","twelve","thirteen","fourteen","fifteen",\
                   "sixteen","seventeen","eighteen","nineteen"]
# Number words that specify the tens digit of a two-digit value.
tens_digit_words = ["twenty","thirty","forty","fifty","sixty","seventy",\
                    "eighty","ninety"]
number_words = one_digit_words + two_digit_words + tens_digit_words
                
male_words = ["men's","men","boy's","boys","boy"]
female_words = ["women's","women","ladies","lady","girl's","girls","girl"]
gender_words = male_words + female_words
                
restroom_words = ["restroom","bathroom","rest","bath"]
                
other_dest_words = ["exit","elevator"]
                    
direction_words = ["north","south"]

office = {"Adali": 324, "Banerjee": 362, "Bargteil": 341, "Carter": 308, \
          "Chen": 357, "Choa": 303, "Desjardins": 337, "Finin": 329, \
          "Halem": 330, "Kim": 312, "Kalpakis": 348, "Laberge": 358, \
          "Lomonaco": 306, "Marron": 359, "Matuszek": 331, "Menyuk": 304, \
          "Mohsenin": 323, "Morris": 308, "Nicholas": 356, "Oates": 336, \
          "Olano": 354, "Patel": 322, "Pearce": 373, "Peng": 327, \
          "Phatak": 319, "Pinkston": 327, "Pirsiavash": 342, "Rheingans": 355, \
          "Robucci": 316, "Schmill": 350, "Sidhu": 347, "Slaughter": 311, \
          "Syed": 301, "Weiss": 302, "Yan": 315, "Younis": 318, "Zhu": 360}
          
# UNCOMMENT THE FOLLOWING LINE TO IGNORE PROFESSOR NAMES
#office = {}

professors = office.keys()
                
dest_words = number_words + gender_words + restroom_words + other_dest_words + \
             direction_words + professors + ["room"]
                             
# Master list of all words to listen for.
watch_words = command_words + dest_words

# Define int values of number words.
value_of = {"zero": 0, "oh": 0, "one": 1, "two": 2, "three": 3, "four": 4, \
            "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, \
            "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, \
            "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18, \
            "nineteen": 19, "twenty": 20, "thirty": 30, "forty": 40, \
            "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90}

class voice_cmd_vel:

  def __init__(self):
    rospy.on_shutdown(self.cleanup)
        
    # Initialize class variables...
    # Rate of refresh.
    self.r = rospy.Rate(10.0)
    # Holds latest word received from recognizer/output
    self.latest_word = ""
    # Indicates whether it's safe for a new word to be written to latest_word.
    self.caught_all = True
    # Mutex for latest_word and caught_all
    self.word_lock = Thread.Lock()

    # publish to husky_nav, subscribe to speech output
    self.pub_ = rospy.Publisher('husky_nav', Int16)
    rospy.Subscriber('recognizer/output', String, self.speechCb)
        
    # Initialize internal variables...
    # Indicates whether robot is currently driving.
    nav_mode = False
    # Indicates whether robot currently has a destination.
    has_dest = False
    
    # Storage space for counts of words in a command that match certain subsets.
    restroom_count = 0
    # Represents the most recent direction mentioned in the command. (1:S, 2:N)
    direction = 0
    # Represents the most recent gender mentioned in the command. (1:F, 2:M)
    gender = 0
    # Storage space for latest word from pocketsphinx.
    new_string = ""
        
    while not rospy.is_shutdown():
      # Main parsing loop...
      # Check for new word.
      self.word_lock.acquire()
      if (self.caught_all):
        # No new word. Release lock.
        self.word_lock.release()
      else:
        # New word. Update caught_all, then parse, then release lock.
        new_string = self.latest_word
        caught_all = True
        word_lock.release()
                
        if (nav_mode):
          # Robot is moving.
          # State is DRIVING (if has_dest) or RUNAWAY (if not has_dest).
          # Behavior: pause if drive_pause or RUNAWAY; otherwise no change.
          if (new_string == drive_pause or not has_dest):
            # new_string == drive_pause: we just received the pause command.
            # not has_dest: we are RUNAWAY and need to generate our own pause command.
                        
            # Tell navigation to stop without discarding destination.
            self.pub_.publish(1)
                        
            # update internal state to PAUSED
            nav_mode = False
        elif (has_dest and new_string == drive_resume):
          # not nav_mode and has_dest: PAUSED.
          # new_string == drive_resume: received resume command.
          # Behavior: resume driving, switch to nav_mode.
                    
          # Pass resume command to navigation stack.
          self.pub_.publish(2)
                    
          # Update internal state to DRIVING.
          nav_mode = True
        elif (new_string == request_start):
          # not (nav_mode): not moving.
          # not (has_dest and new_string == drive_resume): if PAUSED, not resuming.
          # new_string == request_start: receiving new destination command.
          # Behavior: Record and parse incoming destination command.
                    
          # Recording new destination command. Store each word in sequence
          # until command is complete or canceled.
          words = []
          numbers_used = []
          prof_name = ""
          direction = 0
          gender = 0
          restroom_count = 0
          while (new_string != request_cancel and new_string != request_end):
            # Until command is complete (request_end) or discarded
            # (request_cancel), keep listening.
                        
            self.word_lock.acquire()
            if caught_all:
              self.word_lock.release()
            else:
              # Load value from latest_word to new_string.
              new_string = self.latest_word
              self.caught_all = True
              self.word_lock.release()
              # new_string contains a word, so add it to the sequence.
              # Note which category/ies it belongs to.
              words.append(new_string)
              if (new_string == "north"):
                direction = 2
              elif (new_string == "south"):
                direction = 1
              elif (new_string in male_words):
                gender = 2
              elif (new_string in female_words):
                gender = 1
              elif (new_string in number_words):
                numbers_used.append(new_string)
              elif (new_string in restroom_words):
                restroom_count += 1
              elif (new_string in professors):
                prof_name = new_string
              elif (new_string == "room" and gender > 0):
                # "[gender] room"
                restroom_count += 1
            self.r.sleep()
                            
          # If command was canceled, ignore sequence.
          if (new_string != request_cancel):
            # Otherwise, parse sequence.
            if ("exit" in words):
              # User mentioned the exit.
              # Send "exit" identifier (100) to navigation.
              self.pub_.publish(100)
              has_dest = True
              nav_mode = True
            elif ("elevator" in words):
              # User mentioned the elevator.
              # Current value of direction indicates any direction specified.
              value = 110 + direction
              # Send value to navigation.
              self.pub_.publish(value)
              has_dest = True
              nav_mode = True
            elif (restroom_count > 0):
              # User mentioned the restroom.
              # Current value of direction indicates any direction specified.
              # Current value of gender indicates any gender specified.
              value = 120 + 3*direction + gender
              # Send value to navigation.
              self.pub_.publish(value)
              has_dest = True
              nav_mode = True
            elif (prof_name != ""):
              # User mentioned a professor's name.
              # Get that professor's office number.
              room_number = office[prof_name]
              # Pass room_number to navigation.
              self.pub_.publish(room_number)
              has_dest = True
              nav_mode = True
            elif (len(numbers_used) > 0):
              # User mentioned a numbered room.
              # Parse numbers_used into a value.
              result_val = 0
              last_ten = False
              for i in range(0,len(numbers_used)):
                curr_num = numbers_used[i]
                curr_val = value_of[numbers_used[i]]
                if (curr_num in one_digit_words):
                  # e.g., "[result_val] three ..."
                  result_val *= 10
                  result_val += curr_val
                  last_ten = False
                elif (curr_num in two_digit_words):
                  # e.g., "[result_val] twelve ..."
                  if (last_ten):
                    # e.g., "[... thirty] twelve ..."
                    result_val *= 1000
                  else:
                    # e.g., "[... three] twelve ...",
                    #       "[... sixteen] twelve ..."
                    result_val *= 100
                  result_val += curr_val
                  last_ten = False
                elif (curr_num in tens_digit_words):
                  # e.g., "[result_val] forty ..."
                  if (last_ten):
                    # e.g., "[... twenty] forty ..."
                    result_val *= 100
                  else:
                    # e.g., "[... one] forty ...",
                    #       "[... twelve] forty ..."
                    result_val *= 10
                                    
                  result_val += (curr_val/10)
                  last_ten = True
                            
              if (last_ten):
                # e.g., "[... forty]"
                result_val *= 10
                            
              # Check result_val against list of existing room numbers.
              if (result_val >= 300 and result_val <= 377):
                # result_val is a valid third-floor room.
                # Pass result_val to navigation.
                self.pub_.publish(result_val)
                has_dest = True
                nav_mode = True
              else:
                # result_val is a valid number, but not a room on the third
                # floor. Prompt user for new command.
                # ???
                            
            # Done parsing destination command.
            if (not has_dest):
              # Command completed, but didn't include any words that we
              # recognized. Prompt user to try rephrasing.
              # ???
                            
        elif (not has_dest):
          # not nav_mode: not moving.
          # not has_dest: has no destination.
          # State: READY.
          # Prompt user to speak command.
          # ???
            
      # Sleep and wait for the next word.
      self.r.sleep()
        
  def speechCb(self, msg):
    # Callback triggered by new msg from recognizer/output.
    rospy.loginfo(msg.data)
    # Message might not be a single word, so split it into words.
    words_received = msg.data.split()
    # For each word...
    for curr_word in words_received:
      # Wait until the lock is available and the parser has caught all
      # previous words.
      self.word_lock.acquire()
      while (not caught_all):
        self.word_lock.release()
        self.r.sleep()
        self.word_lock.acquire()
      # Move the new word into latest_word.
      self.latest_word = curr_word
      # Indicate by caught_all that there's a new word.
      self.caught_all = False
      # Release the lock and let the parser read the new value.
      word_lock.release()

  def cleanup(self):
    # stop the robot!
    self.pub_.publish(0)

if __name__=="__main__":
  rospy.init_node('voice_cmd_vel')
  try:
    voice_cmd_vel()
  except:
    pass
