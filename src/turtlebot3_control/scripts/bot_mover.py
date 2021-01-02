#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1



def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input


def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel


def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel


class BotMover:
    def __init__(self):
        self.move = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.command = rospy.Subscriber('app_command', String, self.command_callback)
        
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0


    def command_callback(self, msg):
        target_linear_vel   = 0.0
        target_angular_vel  = 0.0
        control_linear_vel  = 0.0
        control_angular_vel = 0.0
        
        button = msg.data

        twist = Twist()
        print(button)
        if button == 'up':          # up
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)

        elif button == 'down':      # down
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)

        elif button == 'left':      # left
            target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)

        elif button == 'right':     # right
            target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)

        elif button == 'stop':      # stop
            #target_linear_vel   = 0.0
            #control_linear_vel  = 0.0
            #target_angular_vel  = 0.0
            #control_angular_vel = 0.0
            #print(vels(target_linear_vel, target_angular_vel))
            self.cmd_vel.linear.x = 0.0 
            self.cmd_vel.angular.z = 0.0
            print(f"currently:linear vel {self.cmd_vel.linear.x}, angular vel {self.cmd_vel.angular.z}")
            self.move.publish(self.cmd_vel)
            return

        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        self.cmd_vel.linear.x += control_linear_vel 

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        self.cmd_vel.angular.z += control_angular_vel
        print(f"currently:linear vel {self.cmd_vel.linear.x}, angular vel {self.cmd_vel.angular.z}")

        self.move.publish(self.cmd_vel)


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node('bot_mover')
    bot = BotMover()
    
    print("r: " + str(ord('r')))
    print("l: " + str(ord('l')))
    print("u: " + str(ord('u')))
    print("d: " + str(ord('d')))
    print("s: " + str(ord('s')))

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
