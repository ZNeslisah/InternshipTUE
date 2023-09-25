#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import sys
import select
import termios
import tty
from threading import Thread


class NeslisahTeleopKey():
    def __init__(self):
        """
        Initialization
        """
        # Flag to indicate if an obstacle is detected
        self.obstacle_detected = False
        self.previous_message = False

        rospy.loginfo("Initializing the NeslisahTeleopKey class...")
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.continuous_control = False # Continuous Command Publication
        self.pause_motion = True
        self.mode = 'pause'
        self.ispressed = False
        self.exit = False
        self.keySettings = termios.tcgetattr(sys.stdin)


    def print_instruction(self):
        """
        Print Control Instructions
        """

        print('***********************************************************')
        print('Improved version of keyboard control by Neslisah')
        print('     w            w/x : increase/descrease linear speed')
        print('  a  s  d         a/d : increase/decrease angular speed')
        print('     x            space key, s : force stop')
        print('                  p/m: pause/move')
        print('                  c/n: continous/noncontinuous\n')
        print('Press <ctrl-c> or <q> to exit')
        print('***********************************************************')

    def print_settings(self): 
        """
        Print current teleoperation settings
        """   
        print('Current Mode: {}, Continuous: {}, Linear Speed: {:.2f}, Angular Speed: {:.2f}'.format(self.mode, self.continuous_control, self.linear_speed, self.angular_speed))

    def getkey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.keySettings)
        return key 
    
    def obstacle_callback(self, msg):
    #Called when a new message is received on 'obstacle_detected' topic.
        if msg.data and not self.previous_message: 
        #Checking if the received message is True and previous message is False
            rospy.logwarn("Obstacle detected! Resetting teleop...")
            self.reset_controls()  # Reset the teleop controls
            self.previous_message = True 
        elif not msg.data:
            self.previous_message = False

    def key_input(self):

        rospy.loginfo("Initializing key_input")

        self.print_instruction()
        self.print_settings()

        kInput = 0
        while not(rospy.is_shutdown() or self.exit):
            key = self.getkey()
            ischanged = False # Flag variable if the steering input is changed
            if key == 'w':
                self.linear_speed = self.linear_speed + 0.1*self.max_linear_speed
                self.linear_speed = min(self.linear_speed, self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'x':
                self.linear_speed = self.linear_speed - 0.1*self.max_linear_speed
                self.linear_speed = max(self.linear_speed, -self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'a':
                self.angular_speed = self.angular_speed + 0.1*self.max_angular_speed
                self.angular_speed = min(self.angular_speed, self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 'd':
                self.angular_speed = self.angular_speed - 0.1*self.max_angular_speed
                self.angular_speed = max(self.angular_speed, -self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 's':
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                self.ispressed = True
                ischanged = True
            if key == 'p':
                self.pause_motion= True
                self.mode = 'pause'
                self.ispressed = False
                ischanged = True
            if key == 'm':
                self.pause_motion = False
                self.mode = 'move '
                self.ispressed = False
                ischanged = True
            if key == 'c':
                self.continuous_control = True
                ischanged = True
            if key == 'n':
                self.continuous_control = False
                ischanged = True
            if (key == 'q') or (key == '\x03'):
                self.exit = True 

            if ischanged:
                kInput = kInput + 1
                if kInput > 10:
                    kInput = 0
                    self.print_instruction()    
                self.print_settings()

    def start(self):
        """
    	Start the unicycle_teleop_key node
    	"""

        rospy.loginfo("Initializing start")

        # Register node with the master
        rospy.init_node('neslisah_teleop_key', anonymous=True)

        rospy.Subscriber('obstacle_detected', Bool, self.obstacle_callback)
        # Create and register control command publisher with the master
        twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist_cmd = Twist()

        # Create and register joy priority publisher with the master
        pause_pub = rospy.Publisher('/pause_motion', Bool, queue_size=1);
        pause_cmd = Bool()

        # Load parameters
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)  # Maximum linear speed
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)  # Maximum angular speed
        cmd_rate = rospy.get_param('~cmd_rate', 10) # Command Publication Rate Parameter

        # Start a thread for keyboard inputs
        key_thread = Thread(target=self.key_input, daemon=True)   
        key_thread.start()

        # Command publication loop
        rate = rospy.Rate(cmd_rate) # Command Publication Rate
        while not(rospy.is_shutdown() or self.exit):
            # Publish command 
            if not(self.pause_motion):
                pause_cmd.data = self.pause_motion
                pause_pub.publish(pause_cmd)
            if not(self.pause_motion) and (self.continuous_control or self.ispressed):
                self.ispressed = False
                twist_cmd.linear.x = self.linear_speed
                twist_cmd.angular.z = self.angular_speed
                twist_pub.publish(twist_cmd)        
            # Loop delay
            rate.sleep()    

    def reset_controls(self):
        #Resets the teleop controls.
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.pause_motion = True
        self.mode = 'pause'
        self.ispressed = False
        self.print_settings()

    
if __name__ == '__main__':
    try:
        #rospy.init_node('neslisah_teleop_key', anonymous=True)
        neslisah_teleop_key = NeslisahTeleopKey().start()
    except rospy.ROSInterruptException:
        pass


