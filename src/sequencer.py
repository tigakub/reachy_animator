#!/usr/bin/env python3

#===============================================================================
# NEW BSD LICENSE

# Copyright 2021 Edward Janne

# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#===============================================================================
# ROS node
# reachy_sequencer/sequencer.py

# A node to play key frame animations from files or published topics.

# Motion blending is currently not implemented. Animations can be triggered
# asynchonously, but must be on independent actuator chains. Simultaneously
# triggering animations that target common actuators will result in
# unpredictable motion.

#===============================================================================
# Format of an animation sequence

# Animation keys are listed one per line and may be any of the following.
# Each line is a series of whitespace separated tokens of the following format,
# or a blank line.

# <key_frame_duration> <command> <parameter*>
#
#   <key_frame_duration> -- floating point time in seconds before the next key
#                           is executed
#   <command>            -- one of dxl, load, or #
#   <parameter*>         -- variable space separated list of parameters

# Command reference

# dxl -- set actuator goal_position
#   <key_frame_duration> dxl <reachy_part_name> <goal_position> <slerp_time>
#
#       <reachy_part_name>  -- string name of the reachy part
#       <goal_position>     -- floating point goal angle in degrees
#       <slerp_time>        -- the period over which to actuate

# load -- load an animation from a file
#   <key_frame_duration> load <full_filepath>
#
#       <full_filepath>     -- the full path name of the animation file to load
#                              (whitespace is not currently permitted)

# # -- comment (can serve to introduce pauses in the animation)
#   <key_frame_duration> # <comment>

# An animation is triggered by publishing a sequence_animation topic, passing
# the sequence as a string

#===============================================================================
# Examples

# Example bash script to use rostopic to play an animation file in the current
# working directory from the commandline
#
#   rostopic pub -1 sequence_animation std_msgs/String "0 load $PWD/$1"

# Example sequence
"""
0 dxl left_arm.shoulder_pitch -90 2
0 dxl left_arm.shoulder_roll 0 1
0 dxl left_arm.hand.wrist_pitch 45 1
0 dxl left_arm.arm_yaw 0 1
0 dxl left_arm.hand.wrist_roll 0 1
1 dxl left_arm.hand.gripper 0 1
1 dxl left_arm.elbow_pitch -90 1.5
0 dxl left_arm.hand.forearm_yaw 45 1.75
1 dxl left_arm.hand.wrist_pitch -45 1
0 dxl left_arm.hand.wrist_pitch -30 0.5
1 dxl left_arm.arm_yaw -30 1.75
1 dxl left_arm.hand.forearm_yaw -45 1.75
1 dxl left_arm.arm_yaw 30 1.75
1 dxl left_arm.hand.forearm_yaw 45 1.75
1 dxl left_arm.arm_yaw -30 1.75
1 dxl left_arm.hand.forearm_yaw -45 1.75
1 dxl left_arm.arm_yaw 30 1.75
1 dxl left_arm.hand.forearm_yaw 45 1.75
1 dxl left_arm.arm_yaw -30 1.75
1 dxl left_arm.hand.forearm_yaw -45 1.75
0 dxl left_arm.shoulder_pitch 0 2
0.5 dxl left_arm.arm_yaw 0 1.5
0.5 dxl left_arm.elbow_pitch 0 1.5
0 dxl left_arm.hand.forearm_yaw 0 2
1 dxl left_arm.hand.wrist_pitch 0 2
0 dxl left_arm.hand.wrist_roll 0 1
"""

# Example of triggering other animations from within an animation
"""
2 load /Users/tigakub/Documents/Projects/CoLab/Reachy/test_scripts/right_wave_animation
0 load /Users/tigakub/Documents/Projects/CoLab/Reachy/test_scripts/left_wave_animation
"""

#===============================================================================

# Import needed modules
import sys
import time
import numpy
import rospy
from threading import Thread, Lock
from collections import OrderedDict
from reachy import Reachy, parts
from reachy.io import IO
from std_msgs.msg import String

def patch_force_gripper(forceGripper):
	def __init__(self, root, io):
		parts.hand.Hand.__init__(self, root=root, io=io)
		dxl_motors = OrderedDict({
			name:dict(conf)
			for name, conf in self.dxl_motors.items()
		})
		self.attach_dxl_motors(dxl_motors)
	forceGripper.__init__ = __init__
	return forceGripper

class Player(Thread):
# Define Player class as derivative of Thread

    def __init__(self, sequence, sequencer):
    # CONSTRUCTOR
        self.alive = False          # flag to indicate thread should stay running
        self.lock = Lock()          # thread protection for above flag
        self.sequence = sequence    # store reference to animation
        self.sequencer = sequencer  # store reference to invoking sequencer object
        super().__init__()          # initialize base object

    def isAlive(self):
    # Thread safe function to return the run status of the thread
        alive = False               # temporary
        self.lock.acquire()         # start critical section
        alive = self.alive          # retrieve value
        self.lock.release()         # exit critical section
        return alive                # return flag

    def abort(self):
    # Thread safe function to clear the run status flag and cause the thread to end
        self.lock.acquire()         # start critical section
        self.alive = False          # clear flag
        self.lock.release()         # exit critical section

    def run(self):
    # Override of Thread.run() that gets executed when Player.start() is called
        self.lock.acquire()         # start critical section
        self.alive = True           # flag thread as running
        self.lock.release()         # end critical section
        currentIndex = 0            # start animation with first keyframe
        # loop while testing protected flag and that reference index is in bounds
        while (self.isAlive() and currentIndex < len(self.sequence)):
            # submit next keyframe for processing and obtain the keyFrame's duration
            keyDuration = self.processKeyFrame(self.sequence[currentIndex])
            # sleep for duration
            if keyDuration > 0:
                time.sleep(keyDuration)
            # increment the keyframe index
            currentIndex += 1
        # when the animation is done, or the thread is aborted, remove self
        self.sequencer.removeSequence(self)

    def processKeyFrame(self, keyFrame):
    # Processes a keyframe
        keyDuration = float(keyFrame[0])        # the duration of this keyFrame
        command = keyFrame[1]                   # the command
        if command == 'dxl': # if this is a motion frame
            partName = keyFrame[2]              # the reachy part name
            goalPosition = float(keyFrame[3])   # the target angle
            goalPeriod = float(keyFrame[4])     # the period over which to slerp
            # submit goal to the named reachy part
            self.sequencer.reachy.goto({
                    partName: goalPosition
                },
                duration = goalPeriod,
                starting_point='present_position',
                wait=False,                             # asynchronous
                interpolation_mode='minjerk')           # motion easing
        elif command == 'load': # if this is a load animation file frame
            filePath = keyFrame[2]                      # the file path
            self.sequencer.loadSequence(filePath)       # tell sequencer to load the file
        elif command == 'log':
            logMsg = keyFrame[2]                        # the message to log
            rospy.loginfo(logMsg)                       # echo message to the log
        elif command == '#': # if this is a comment
            pass                                        # do nothing
        return keyDuration

class Sequencer:
# Define Sequencer class to interface with reachy, and sequence published animations

    def __init__(self, right_arm_io='ws', left_arm_io='ws', name='reachy_sequencer'):
    # CONSTRUCTOR
        rospy.init_node(name, anonymous=True)   # initialize the ros node

        self.lio = left_arm_io                  # store the left connection reference
        self.rio = right_arm_io			# store the right arm connection reference
        self.name = name                        # store the node name
        self.players = list()                   # a list to store current players
        self.playerLock = Lock()                # thread protection for the player list

        self.reachy = Reachy(                   # instantiate Reachy interface
            right_arm=parts.RightArm(io=self.rio, hand='force_gripper'),
            left_arm=parts.LeftArm(io=self.lio, hand='force_gripper'))

        for m in self.reachy.right_arm.motors:
            m.compliant = False

        for m in self.reachy.left_arm.motors:
            m.compliant = False
	
        # subscrive to sequence_filepath topic
        self.sequenceSubscriber = rospy.Subscriber(
                'sequence_animation',
                String,
                self.queueSequence
            )

        rospy.spin()                            # spin until sigint


    def queueSequence(self, msg):
    # Function to sequence an animation received directly as a topic
        animation = msg.data                # get topic msg data (String)
        sequence = list()                   # list to store keyframes
        for line in animation.split('\n'):  # iterate through lines in string
            tokens = line.split()           # split line into tokens
            if len(tokens) > 0:             # only add if there's something to add
                sequence.append(tokens)     # append token list to sequence list

        self.playSequence(sequence)         # call function to start playing

    def loadSequence(self, fileName):
    # Function to load an animation from a file and sequence it
        sequence = list()                   # list to store keyframes
        with open(fileName, 'r') as f:      # open a file
            for line in f:                  # iterate through file line by line
                tokens = line.split()       # split line into tokens
                sequence.append(tokens)     # append token list to sequence list

        self.playSequence(sequence)         # call function to start playing

    def playSequence(self, sequence):
    # Function to instantiate a Player object to play a sequence
        player = Player(sequence, self)     # instantiate Player object
        self.playerLock.acquire()           # enter critical section
        self.players.append(player)         # store player
        self.playerLock.release()           # exit critical section
        player.start()                      # start thread execution

    def removeSequence(self, player):
    # Function to remove Player object from list
        self.playerLock.acquire()           # enter critical section
        self.players.remove(player)         # remove player
        self.playerLock.release()           # exit critical section

    def stopAll(self):
    # Function to kill a playing animations
        self.playerLock.acquire()           # enter critical section
        for p in players:                   # iterate through current players
            p.abort()                       # kill each
        self.playerLock.release()           # exit critical section

if __name__ == '__main__':
# Main entry point
    parts.arm.RightForceGripper = patch_force_gripper(parts.arm.RightForceGripper)
    parts.arm.LeftForceGripper = patch_force_gripper(parts.arm.LeftForceGripper)
   
    Sequencer(right_arm_io='/dev/ttyUSB3', left_arm_io='/dev/ttyUSB2')                             # start the node
