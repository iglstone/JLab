#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# this class is main control the car move by ps joy, not arms.

class Ps3Control(object):
    def __init__(self):
        self.cmd = None
        rospy.init_node("ps3joy_control_node")
        rospy.Subscriber("/joy", Joy, self.callback)
        rospy.on_shutdown(self.shutdown)
        self.velpub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(50)
        self.car_vmax = 0.4  # m/s
        self.speed = 0.1
        self.current = rospy.get_rostime()
        self.last = rospy.get_rostime()
        self.is_change = False
        while not rospy.is_shutdown():
            rate.sleep()
            if self.cmd:
                self.velpub.publish(self.cmd)

    def shutdown(self):
        cmd2 = Twist()
        self.velpub.publish(cmd2)  # stop the robot
        rospy.loginfo('shut down now..')

    def callback(self, data):
        # rospy.loginfo(data)
        ccmd = Twist()

        scal_vx = data.axes[0]
        scal_vy = data.axes[1]

        if not self.is_change:
            if data.buttons[8] == 1:
                if self.speed > 0.1:
                    self.speed = self.speed - 0.1
                    self.is_change = True
                    if self.speed < 0.1:
                        self.speed = 0.1
            if data.buttons[10] == 1:
                if self.speed < 0.4:
                    self.speed = self.speed + 0.1
                    self.is_change = True
                    if self.speed > 0.4:
                        self.speed = 0.4

        self.current = rospy.get_rostime()
        if (self.current - self.last).to_sec() > 0.5:
            self.is_change = False
            self.last = self.current

        '''
        self.current = rospy.get_rostime()
        if (self.current - self.last).to_sec() > 0.5:
            if data.buttons[8] == 1:
                if self.speed > 0.1:
                    self.speed = self.speed - 0.1
                    if self.speed < 0.1:
                        self.speed = 0.1
            if data.buttons[10] == 1:
                if self.speed < 0.4:
                    self.speed = self.speed + 0.1
                    if self.speed > 0.4:
                        self.speed = 0.4
        self.last = self.current
        '''

        scal_horizen = data.axes[2]
        if scal_horizen < -0.5:
            ccmd.angular.z = self.speed
        if scal_horizen > 0.5:
            ccmd.angular.z = -1 * self.speed

        threash_hold = 0.05
        if (scal_vx > threash_hold or scal_vx < -1 * threash_hold) or (scal_vy > threash_hold or scal_vy < -1 * threash_hold):
            ccmd.linear.y = scal_vx * self.car_vmax
            ccmd.linear.x = scal_vy * self.car_vmax
            self.cmd = ccmd
            return

        if data.buttons[4] == 1 or data.buttons[6] == 1:  # go line
            if data.buttons[4] == 1:
                ccmd.linear.x = self.speed
            else:
                ccmd.linear.x = -1 * self.speed
        else:
            ccmd.linear.x = 0  # important,for button released
        if data.buttons[5] == 1 or data.buttons[7] == 1:  # rotate
            if data.buttons[7] == 1:
                ccmd.linear.y = self.speed
            else:
                ccmd.linear.y = -1 * self.speed
        else:
            ccmd.linear.y = 0

        self.cmd = ccmd


if __name__ == '__main__':
    Ps3Control()

# <launch>
#   <node pkg="learning_ps3joy" type="listener.py" name="learningPs3">
#     <remap from="/jbot/cmd_vel" to="/cmd_vel"/>
#     <remap from="/jbot/joy" to="/joy"/>
#   </node>
#
#   <node pkg="joy" type="joy_node" name="joystick"/>
# </launch>
