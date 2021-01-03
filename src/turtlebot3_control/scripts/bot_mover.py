#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1



class BotMover:
    def __init__(self):
        self.move = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.command = rospy.Subscriber('app_command', String, self.command_callback)
        
        self.max_lin = BURGER_MAX_LIN_VEL
        self.max_ang = BURGER_MAX_ANG_VEL
        
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x  = 0.0
        self.cmd_vel.linear.y  = 0.0
        self.cmd_vel.linear.z  = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0


    def command_callback(self, msg):
        
        button = msg.data
        
        if button == 'up':          # up
            self.cmd_vel.linear.x = self.constrain(self.cmd_vel.linear.x + LIN_VEL_STEP_SIZE, "linear")

        elif button == 'down':      # down
            self.cmd_vel.linear.x = self.constrain(self.cmd_vel.linear.x - LIN_VEL_STEP_SIZE, "linear")

        elif button == 'left':      # left
            self.cmd_vel.angular.z = self.constrain(self.cmd_vel.angular.z + ANG_VEL_STEP_SIZE, "angular")

        elif button == 'right':     # right
            self.cmd_vel.angular.z = self.constrain(self.cmd_vel.angular.z - ANG_VEL_STEP_SIZE, "angular") 

        elif button == 'stop':      # stop
            self.cmd_vel.linear.x = 0.0 
            self.cmd_vel.angular.z = 0.0
            print("Stopping the movement!")
            self.move.publish(self.cmd_vel)
            return

        else:
            return

        print(f"Currently: linear vel {self.cmd_vel.linear.x}, angular vel {self.cmd_vel.angular.z}")

        self.move.publish(self.cmd_vel)


    def constrain(self, input_value, vel_type):
        if vel_type == "angular":
            low  = -self.max_ang
            high =  self.max_ang 
        else:
            low  = -self.max_lin
            high =  self.max_lin

        if input_value < low:
            return low
        elif input_value > high:
            return high
        else:
            return input_value


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node('bot_mover')
    bot = BotMover()
    
    try:
        bot.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        twist = Twist()
        twist.linear.x = 0.0 
        twist.linear.y = 0.0 
        twist.linear.z = 0.0
        twist.angular.x = 0.0 
        twist.angular.y = 0.0 
        twist.angular.z = 0.0
        bot.move.publish(twist)
