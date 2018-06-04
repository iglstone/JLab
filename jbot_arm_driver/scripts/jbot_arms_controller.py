#!/usr/bin/env python

import rospy
import threading
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import serial
import struct

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
JOINT_NAMES_LEFT = ['joint_11', 'joint_21', 'joint_31', 'joint_41', 'joint_51']
PI = 3.141593
req_time_duration = 0.2  # 0.2  # 0.03


class JBotArmsController(object):
    def __init__(self, port='/dev/ttyUSB_right_arm', joint_names=[], gripper_topic='/cmd_gripper'):

        self._joint_names = joint_names
        self._gripper_topic = gripper_topic

        self.__ser = serial.Serial(port, 9600, timeout=0.5)
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

        self.joint_states = JointState()
        self.joint_states.name = self._joint_names  # JOINT_NAMES

        self.__ros_pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=3)
        self.__ros_sub_control_arms = rospy.Subscriber(self._gripper_topic, Int8, self.__gripper_control)

        self.thread_joint_state_req = threading.Thread(target=self.__joint_states_request)
        self.thread_receive = threading.Thread(target=self.__serial_receive)
        self.thread_receive.setDaemon(True)
        self.thread_joint_state_req.setDaemon(True)

    def start(self):
        while not rospy.is_shutdown():
            self.thread_joint_state_req.start()
            self.thread_receive.start()

    def __joint_states_request(self):
        # data = b"\x55\x55\x09\x15\x06\x01\x02\x03\x04\x05\x06"  # 6 joints
        data = b"\x55\x55\x08\x15\x05\x02\x03\x04\x05\x06"  # 5 joints
        while True:
            if self.__flag.isSet():
                rospy.sleep(req_time_duration)
                self.__ser.write(data)
                # print(repr(data))
            else:
                print "flag will wait"
                self.__flag.wait()

    def cmd_control_arm_init_position(self):
        # move the arm joints 1-5 to PI/2 positinos in 1000ms
        data = b"\x55\x55\x14\x03\x05\x88\x13\x06\xf4\x01\x05\xf4\x01\x04\xf4\x01\x03\xf4\x01\x02\xf4\x01"
        # print(repr(data))
        self.__ser.write(data)
        rospy.sleep(5)

    # moveit driver interface
    def cmd_control_arm_trajectory(self, ppositions=[]):
        # time_ms = 500
        time_ms = 3500
        byte_com2 = struct.pack('<BBBBBH', 85, 85, 20, 3, 5, time_ms)
        byte_com = ''
        joint_nums = [0, 1, 2, 3, 4]
        result = 0
        for (joint_num, position) in zip(joint_nums, ppositions):
            if joint_num == 0 or joint_num == 4:
                if position > (PI + 0.05) or position < (-PI - 0.05):
                    print 'Target position args wrong! ' \
                          'joint_num:{0} position:{1}'.format(str(joint_num), str(position))
                    return result
            else:
                if position > (PI / 2 + 0.05) or position < (-PI / 2 - 0.05) or joint_num > 5:
                    print 'Target position args wrong! ' \
                          'joint_num:{0} position:{1}'.format(str(joint_num), str(position))
                    return result

            position = (position / PI * 1000.0) + 500  # from rad to servor joint
            position = int(position)
            if position < 0:
                position = 0

            joint_num = 6 - joint_num  # moveit joint 2-6  -> servor id 4-0

            # cotorl the servo move to the target position in 1000 ms
            try:
                byte = struct.pack('<BH', joint_num, position)
            except struct.error, msg:
                print 'struct error msg : {0} servor id:{1} servor rad:{2}'.format(msg, str(joint_num), str(position))
                return result
            byte_com = byte_com + byte

        com = byte_com2 + byte_com
        data = bytes(com)
        # print(repr(data))
        self.__ser.write(data)
        # rospy.sleep(time_ms/1000.0 + 0.05)
        return 1

    def cmd_control_one_joint(self, index, method):
        joint_nums = [index]
        position_joint = self.joint_states.position[index]
        if index == 0:
            if position_joint > PI or position_joint < -PI:
                print 'Target position args wrong! {0} {1}'.format(str(index), str(position_joint))
                return
        elif index < 5:
            if position_joint > (PI/2 + 0.05) or position_joint < (-PI/2 - 0.05):
                print 'Target position args wrong! {0} {1}'.format(str(index), str(position_joint))
                return
        else:
            print('index out of 5 {0}'.format(str(index)))
            return
        if method % 2 == 0:
            position_joint += 0.1  # add 0.1rad per time
        else:
            position_joint -= 0.1
        pposition = [position_joint]
        self.cmd_control_arm_target_positions(joint_nums, pposition)

    # moveit driver interface
    def cmd_control_arm_target_positions(self, joint_nums=[], ppositions=[]):
        byte_com = ''
        for (joint_num, position) in zip(joint_nums, ppositions):
            print 'joint_num: {0}  position: {1}'.format(str(joint_num), str(position))
            if position > (PI/2 + 0.05) or position < (-PI/2 - 0.05) or joint_num > 5:
                print 'Target position args wrong! {0} {1}'.format(str(joint_num), str(position))
                return
            position = (position/PI * 1000.0) + 500  # from rad to servor joint
            position = int(position)

            joint_num = 6 - joint_num  # moveit joint 2-6  -> servor id 4-0

            # cotorl the servo move to the target position in 1000 ms
            byte = struct.pack('<BH', joint_num, position)
            byte_com = byte_com + byte

        servo_num = len(joint_nums)
        bytes_num = servo_num * 3 + 5
        time_ms = 1000
        byte_com2 = struct.pack('<BBBBBH', 85, 85, bytes_num, 3, servo_num, time_ms)

        com = byte_com2 + byte_com

        data = bytes(com)
        # print(repr(data))
        self.__flag.clear()
        rospy.sleep(0.2)
        self.__ser.write(data)
        self.__flag.set()

    def cmd_control_arm_target_index_position(self, index):
        self.__flag.clear()
        rospy.sleep(0.5)
        data = ''
        if index == 1:
            # move the arm joints 1-5 to PI/2 positinos in 1000ms
            data = b"\x55\x55\x14\x03\x05\x88\x13\x06\xf4\x01\x05\xf4\x01\x04\xf4\x01\x03\xf4\x01\x02\xf4\x01"
        elif index == 2:
            data = b"\x55\x55\x08\x03\x01\x88\x13\x03\xc8\x00"
        elif index == 3:
            data = b"\x55\x55\x08\x03\x01\x88\x13\x04\xc8\x00"
        elif index == 4:
            data = b"\x55\x55\x08\x03\x01\x88\x13\x05\xc8\x00"
        # print(repr(data))

        self.__ser.write(data)
        self.__flag.set()
        if self.__flag.is_set():
             print 'flag is true'

    def gripper_control(self, data):
        self.__flag.clear()
        rospy.sleep(0.05)
        data_st = ''
        if data % 2 == 0:
            # data_st = b"\x55\x55\x08\x03\x01\xe8\x03\x01\xd0\x07"  # open
            data_st = b"\x55\x55\x08\x03\x01\xe8\x03\x01\xd0\x06"
        else:
            # data_st = b"\x55\x55\x08\x03\x01\xe8\x03\x01\xd0\x00"  # close
            data_st = b"\x55\x55\x08\x03\x01\xe8\x03\x01\xd0\x02"
        self.__ser.write(data_st)
        self.__flag.set()

    def __gripper_control(self, data):
        self.__flag.clear()
        rospy.sleep(0.05)
        if data.data == 0:
            self.cmd_control_arm_target_index_position(1)
        else:
            if data.data % 2 == 0:
                data_0pen = b"\x55\x55\x08\x03\x01\xe8\x03\x01\xd0\x07"
                self.__ser.write(data_0pen)
            else:
                data_close = b"\x55\x55\x08\x03\x01\xe8\x03\x01\xd0\x00"
                self.__ser.write(data_close)
        self.__flag.set()

    def __recv_begain_with(self):
        # data start with 'UU'
        while True:
            data = self.__ser.read(1)
            if data != 'U':
                continue
            else:
                data = self.__ser.read(1)
                if data != 'U':
                    continue
                else:
                    break
        rcv = self.__ser.read(18)  # 21
        return rcv

    def __serial_receive(self):

        while True:
            # rcv = self.__ser.read(20)
            rcv = self.__recv_begain_with()
            reprcv = bytes(rcv)
            # print 'serial rcv ....: ', repr(reprcv)

            if len(reprcv) != 18:  # 21
                print ('rcv len: %d' % len(reprcv))
                rospy.sleep(req_time_duration)
                continue

            intStr = struct.unpack('<BBBBHBHBHBHBH', reprcv)  # unsigned short, 5 joints

            joints_p = []
            for i in range(12, 3, -2):
                joint_i = intStr[i] / 1000.0 * PI - 0.5 * PI
                joints_p.append(joint_i)

            # print(joints)
            rospy.sleep(req_time_duration/2)

            self.joint_states.position = joints_p
            self.joint_states.header.stamp = rospy.get_rostime()
            self.joint_states.header.seq += 1
            self.__ros_pub_joint_states.publish(self.joint_states)


if __name__ == '__main__':
    rospy.init_node("JBot_Arms_Controller")
    try:
        _hrg_controller = JBotArmsController(
            port='/dev/ttyUSB_left_arm', joint_names=JOINT_NAMES, gripper_topic='cmd_gripper')
        _hrg_controller.thread_joint_state_req.start()
        _hrg_controller.thread_receive.start()

        _hrg_controller_left = JBotArmsController(
            port='/dev/ttyUSB_right_arm', joint_names=JOINT_NAMES_LEFT, gripper_topic='cmd_gripper_left')
        _hrg_controller_left.thread_joint_state_req.start()
        _hrg_controller_left.thread_receive.start()

        # _hrg_controller.cmd_control_arm_target_index_position(1)
        # _hrg_controller_left.cmd_control_arm_target_index_position(1)

        for t in range(0, 3):
            for i in range(1, 5, 1):
                _hrg_controller.cmd_control_arm_target_index_position(i)
                rospy.sleep(8)
                _hrg_controller_left.cmd_control_arm_target_index_position(i)
                rospy.sleep(8)

        '''
        for t in range(0, 15):
            _hrg_controller.cmd_control_arm_target_index_position(1)
            rospy.sleep(2)
            ax = [3, 4]
            positions = [0.25*PI, -0.25*PI]
            _hrg_controller.cmd_control_arm_target_positions(ax, positions)
            rospy.sleep(3)
        '''
        while not rospy.is_shutdown():
            print("main thread ")
            rospy.sleep(0.1)
            rospy.spin()

    except Exception.message:
        print "...........xxx............", Exception.message
        pass
