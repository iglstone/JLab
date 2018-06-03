#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from jbot_arms_controller import JBotArmsController
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
JOINT_NAMES_LEFT = ['joint_11', 'joint_21', 'joint_31', 'joint_41', 'joint_51']

MODE_JOINT = 100
MODE_SLIDER = 101
SLIDER_UP = 1001
SLIDER_STOP = 1002
SLIDER_DOWN = 1003


class JBotArmsDriver(object):
    def __init__(self):
        rospy.init_node("Xbot_Robot_Arms_Driver")

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.leftOrRight = 0
        self.method = 0
        self.open = 0

        self.rate = 20

        self.controller_left = JBotArmsController(
            port='/dev/ttyUSB_left_arm', joint_names=JOINT_NAMES_LEFT, gripper_topic='/cmd_gripper_left')
        self.controller_left.thread_joint_state_req.start()
        self.controller_left.thread_receive.start()

        self.controller_right = JBotArmsController(
            port='/dev/ttyUSB_right_arm', joint_names=JOINT_NAMES, gripper_topic='/cmd_gripper')
        self.controller_right.thread_joint_state_req.start()
        self.controller_right.thread_receive.start()

        self.controller = self.controller_right
        self.controller.cmd_control_arm_target_index_position(1)
        self.controller_left.cmd_control_arm_target_index_position(1)

        # for t in range(0, 1):
        #     self.controller.cmd_control_arm_target_index_position(1)
        #     rospy.sleep(6)
        #     ax = [3, 4]
        #     positions = [0.25*PI, -0.25*PI]
        #     self.controller.cmd_control_arm_target_positions(ax, positions)

        # action server
        self.server_right = actionlib.SimpleActionServer('/right_arm_controller/follow_joint_trajectory',
                                                         FollowJointTrajectoryAction,
                                                         execute_cb=self.actionCb_right,
                                                         auto_start=True)

        self.server_left = actionlib.SimpleActionServer('/left_arm_controller/follow_joint_trajectory',
                                                        FollowJointTrajectoryAction,
                                                        execute_cb=self.actionCb_left,
                                                        auto_start=True)

        self.mode = MODE_JOINT
        self.slider_state = SLIDER_STOP

        self.__pub_slider_states = rospy.Publisher('/slider_states', Int16, queue_size=3)

        rospy.loginfo("Started arm driver")

        while not rospy.is_shutdown():
            print("main thread ")
            rospy.sleep(0.1)
            rospy.spin()

    def joy_callback(self, data):
        if data.buttons[0] == 1:
            self.leftOrRight += 1
            if self.leftOrRight % 2 == 0:
                self.controller = self.controller_right
            else:
                self.controller = self.controller_left

        if data.buttons[3] == 1:  # start button, change the Mode
            if self.mode == MODE_JOINT:
                self.mode = MODE_SLIDER
            else:
                self.mode = MODE_JOINT

        scal_vertical = data.axes[3]
        if self.mode == MODE_JOINT:  # control joint
            if scal_vertical > 0.5:
                self.method = 2
            if scal_vertical < -0.5:
                self.method = 1

            if data.buttons[9] == 1:
                self.controller.gripper_control(self.open)
                self.open += 1
            if data.buttons[11] == 1:
                self.controller.cmd_control_one_joint(4, self.method)
            if data.buttons[12] == 1:
                self.controller.cmd_control_one_joint(3, self.method)
            if data.buttons[13] == 1:
                self.controller.cmd_control_one_joint(2, self.method)
            if data.buttons[14] == 1:
                self.controller.cmd_control_one_joint(1, self.method)
            if data.buttons[15] == 1:
                self.controller.cmd_control_one_joint(0, self.method)
        else:  # control slider
            if scal_vertical > 0.5:
                self.slider_state = SLIDER_UP
            elif scal_vertical < -0.5:
                self.slider_state = SLIDER_DOWN
            else:
                self.slider_state = SLIDER_STOP
            self.__pub_slider_states.publish(self.slider_state)

    def actionCb_right(self, goal):
        traj = goal.trajectory
        rospy.loginfo("...........Action goal recieved.. %s" % str(traj))

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server_right.set_aborted(text=msg)
            return

        '''
        try:
            indexes = [traj.joint_names.index(joint.name) for joint in JOINT_NAMES]
        except ValueError as val:
            msg = "Trajectory invalid. info: {0}".format(str(val))
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        print 'indexes: {0}'.format(str(indexes))

        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()

        r = rospy.Rate(self.rate)
        '''

        lenth = len(traj.points)
        pp = list()
        # mid = lenth / 2
        # pp.append(traj.points[mid])
        pp.append(traj.points[lenth-1])
        print('traj:{0} '.format(str(pp)))

        # for point in traj.points:
        for point in pp:
            positions = point.positions
            re = self.controller_right.cmd_control_arm_trajectory(positions)
            if re == 0:
                self.server_right.set_aborted(text='Something wrong with robot excute')
                return

        self.server_right.set_succeeded(text='Trjectory excu success')

    def actionCb_left(self, goal):
        traj = goal.trajectory
        rospy.loginfo("...........Action goal recieved.. %s" % str(traj))

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server_left.set_aborted(text=msg)
            return

        lenth = len(traj.points)
        pp = list()
        # mid = lenth / 2
        # pp.append(traj.points[mid])
        pp.append(traj.points[lenth - 1])
        print('traj:{0} '.format(str(pp)))

        # for point in traj.points:
        for point in pp:
            positions = point.positions
            re = self.controller_left.cmd_control_arm_trajectory(positions)
            if re == 0:
                self.server_left.set_aborted(text='Something wrong with robot excute')
                return

        self.server_left.set_succeeded(text='Trjectory excu success')


if __name__ == '__main__':
    try:
        driver = JBotArmsDriver()

    except Exception.message:
        print "...........xxx............", Exception.message
        pass
