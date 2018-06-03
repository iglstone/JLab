#!/usr/bin/env python

import rospy
import threading
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
import serial
from math import cos, sin
from tf import TransformBroadcaster, transformations
from std_msgs.msg import Int16
import struct

PI = 3.1416
TIMER_OUT = 0.02  # 0.015


class JBotBaseDriver(object):
    def __init__(self, port='/dev/ttyUSB0', vel_topic='/cmd_vel'):

        self._vel_topic = vel_topic

        self.__ser = serial.Serial(port, 115200, timeout=TIMER_OUT)
        self.__ser.bytesize = 8
        self.__ser.parity = serial.PARITY_NONE
        self.__ser.stopbits = 1
        if self.__ser.isOpen():
            self.__ser.close()
        try:
            self.__ser.open()
        except serial.SerialException, msg:
            print 'Serial open failed! :' + msg
            sys.exit()

        self.__flag = threading.Event()
        self.__flag.set()

        self.joint_state_slider = rospy.Publisher('/joint_states', JointState, queue_size=3)
        self.__ros_pub_odomtry = rospy.Publisher('/odom', Odometry, queue_size=3)
        self.__ros_sub_velocity = rospy.Subscriber(self._vel_topic, Twist, self.__vel_control)
        self.odom_broadcaster = TransformBroadcaster()
        self.__sub_slider_states = rospy.Subscriber('/slider_states', Int16, self.__slider_callback)

        self.thread_send_cmdvel = threading.Thread(target=self.__send_cmdvel)
        self.thread_receive = threading.Thread(target=self.__serial_receive)
        self.thread_receive.setDaemon(True)
        self.thread_send_cmdvel.setDaemon(True)

        self.vh = 1002
        self.fm_cmd_vel = '`0|0|0|1002|0~\r'
        self.mutex = threading.Lock()

        self.r = rospy.Rate(50)
        # self.r = rospy.Rate(3)

    #  send cmd vel evey 20 ms, 50hz
    def __send_cmdvel(self):
        time_sleep = TIMER_OUT  # 0.015  # send cmd vel evey 20 ms, 50hz
        while True:
            # print('send cmd_vel: {0}'.format(self.fm_cmd_vel))
            self.__ser.write(self.fm_cmd_vel)
            rospy.sleep(time_sleep)

    def __slider_callback(self, data):
        slider_state = data.data
        self.vh = slider_state

    def __vel_control(self, data):
        vx = data.linear.x
        vy = data.linear.y
        vz = data.angular.z
        # as protocol , need * 1000 and then pass to sub layer
        vx = int(1000 * vx)
        vy = int(1000 * vy)
        vz = int(1000 * vz)
        vz = -vz
        vh = self.vh

        # self.fm_cmd_vel = '`{0}|{1}|{2}~'.format(str(vx), str(vy), str(vz))
        # self.__ser.write(self.fm_cmd_vel)

        self.mutex.acquire()
        # self.fm_cmd_vel = '`{0}|{1}|{2}|0~\r'.format(str(vy), str(vx), str(vz))
        self.fm_cmd_vel = '`{0}|{1}|{2}|{3}|0~\r'.format(str(vy), str(vx), str(vz), str(vh))
        # byte = struct.pack('>ciii', 72, vx, vy, vz)  # big edian, start with H
        # print('sent cmd_vel : {0}'.format(str(self.fm_cmd_vel)))
        self.mutex.release()

    def __serial_receive(self):
        slider_joint_state = JointState()
        slider_joint_state.name = ['joint_slider']  # JOINT_NAMES

        current_time = rospy.get_rostime()
        last_time = rospy.get_rostime()
        x = 0.0
        y = 0.0
        th = 0.0
        vx_1 = 0
        vy_1 = 0
        vz_1 = 0

        scal_x = 0.642  # 0.68
        scal_y = 0.642  # 0.62
        scal_th = 0.624  # 0.71

        print("Start to receive data from the base control board..")
        count = 0
        count_speed = 0
        thresh_hold = 0.12  # 0.1 * 0.62
        while True:
            rcv = self.__ser.readline()  # need TimeOut = 0.02ms, 50hz
            if len(rcv) == 0:
                continue
            # print("xxxxx: " + str(rcv))
            # self.r.sleep()
            # continue

            if rcv.startswith('`') and rcv.endswith('~\r\n'):
                # print 'rcv : {0}'.format(rcv)
                rcv_new = rcv[1:len(rcv) - 3]
                rcv_list = rcv_new.split('|')
                if len(rcv_list) == 4:
                    vy = float(rcv_list[0]) / 1000 * scal_x
                    vx = float(rcv_list[1]) / 1000 * scal_y
                    vz = float(rcv_list[2]) / 1000 * scal_th

                    '''
                    # filter data
                    # if new one much bigger than old one, not update
                    if abs(vx - vx_1) > thresh_hold:  # or abs(vy_1 - vy) > thresh_hold or abs(vz_1 - vz) > thresh_hold:
                        # vx = vx_1
                        # vy = vy_1
                        # vz = vz_1
                        print ('..........out of the thresh hold')
                        vx = vx_1  # not update
                        # continue
                    else:
                        vx_1 = vx  # update old one
                        vy_1 = vy
                        vz_1 = vz
                    '''

                    slide_height = 0.5  # float(rcv_list[3])
                    if vx != 0 or vy != 0 or vz != 0:
                        count_speed = count_speed + 1
                        if count_speed / 20 == 1:
                            print('rcv vx: {0}, vy: {1}, vz: {2}'.format(str(vx), str(vy), str(vz)))
                            count_speed = 0
                    else:
                        count = count + 1
                        if count / 120 == 1:
                            print('vx: {0}, vy: {1}, vz: {2}'.format(str(vx), str(vy), str(vz)))
                            count = 0

                    # if vy == 0 and vx == 0 and vz == 0:continue

                    slider_joint_state.position = [slide_height]
                    slider_joint_state.header.stamp = rospy.get_rostime()
                    slider_joint_state.header.seq += 1
                    self.joint_state_slider.publish(slider_joint_state)

                    current_time = rospy.get_rostime()
                    dt = (current_time - last_time).to_sec()
                    # print ('dt: {0}, {1} ,{2}'.format(str(dt), str(current_time), str(last_time)))
                    delta_x = (vx * cos(th) - vy * sin(th)) * dt
                    delta_y = (vx * sin(th) + vy * cos(th)) * dt

                    delta_th = -vz * dt

                    x += delta_x
                    y += delta_y
                    th += delta_th
                    if th > PI:
                        th = th - PI * 2
                    if th < -PI:
                        th = th + PI * 2

                    '''
                        euler = transformations.quaternion_from_euler(0, 0, -th)
                        odom_quat = Quaternion(*euler)
                    '''

                    odom_quat = Quaternion()
                    odom_quat.x = 0.0
                    odom_quat.y = 0.0
                    odom_quat.z = sin(th / 2)
                    odom_quat.w = cos(th / 2)

                    odom_trans = TransformStamped()
                    odom_trans.header.stamp = current_time
                    odom_trans.header.frame_id = "odom"
                    odom_trans.child_frame_id = "base_footprint"

                    odom_trans.transform.translation.x = x
                    odom_trans.transform.translation.y = y
                    odom_trans.transform.translation.z = 0.0
                    odom_trans.transform.rotation = odom_quat

                    self.odom_broadcaster.sendTransformMessage(odom_trans)

                    odom = Odometry()
                    odom.header.stamp = current_time
                    odom.header.frame_id = 'odom'

                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.y = y
                    odom.pose.pose.position.z = 0.0
                    odom.pose.pose.orientation = odom_quat

                    odom.child_frame_id = 'base_footprint'
                    odom.twist.twist.linear.x = vx
                    odom.twist.twist.linear.y = vy
                    odom.twist.twist.angular.z = vz

                    self.__ros_pub_odomtry.publish(odom)

                    last_time = current_time
                    self.r.sleep()

                else:
                    print('rcv size wrong! new size:{0}'.format(str(len(rcv_list))))
                    continue
            else:
                # print('rcv format wrong!rcv:{0}'.format(rcv))
                continue


if __name__ == '__main__':
    rospy.init_node("JBot_Base_Driver")

    try:
        base_driver = JBotBaseDriver(
            port='/dev/ttyUSB_base_control', vel_topic='/cmd_vel')

        base_driver.thread_receive.start()
        base_driver.thread_send_cmdvel.start()

        while not rospy.is_shutdown():
            print("main thread ")
            rospy.sleep(0.1)
            rospy.spin()

    except Exception.message:
        print "...........xxx............", Exception.message
        pass
