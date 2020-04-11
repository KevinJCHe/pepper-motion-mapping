#!/usr/bin/env python

from naoqi import ALProxy
import qi

import rospy
from joint_converter_msgs.msg import PepperJoint
from sensor_msgs.msg import JointState

from threading import Thread

import pandas as pd
import time

class MovePepper:

    def __init__(self, only_sim):
        self.only_simulation = only_sim
        self.pepper_sub = rospy.Subscriber("/pepper_joint_angles", PepperJoint, self.callback_pepper)
        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.joint_names = ['HeadYaw', 'HeadPitch', 'HipRoll', 'HipPitch', 'KneePitch', 
                            'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 
                            'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 
                            'RFinger41', 'LFinger42', 'RFinger12', 'LFinger33', 'RFinger31', 
                            'LFinger21', 'RFinger32', 'LFinger13', 'LFinger32', 'LFinger11', 
                            'RFinger22', 'RFinger13', 'LFinger22', 'RFinger21', 'LFinger41', 
                            'LFinger12', 'RFinger23', 'RFinger11', 'LFinger23', 'LFinger43', 
                            'RFinger43', 'RFinger42', 'LFinger31', 'RFinger33', 'LThumb1', 
                            'RThumb2', 'RThumb1', 'LThumb2', 
                            'WheelFL', 'WheelB', 'WheelFR']
        self.joint_idx = {joint: idx for idx, joint in enumerate(self.joint_names)}
        self.joint_state = JointState()
        self.joint_state.name = self.joint_names
        self.joint_state.position = [0.0 for joint in self.joint_names]

        self.motion_names = ['HeadPitch', 'HeadYaw', 
                             'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
                             'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll']
        size = len(self.motion_names)
        self.motion_keys = [0.0 for i in range(size)]
        self.prev_motion_keys = self.motion_keys
        self.motion_times = [0.5 for i in range(size)]

        if not self.only_simulation:
            self.pepper_ip = "pepper.local"
            self.pepper_port = 9559
            self.motion = ALProxy("ALMotion", self.pepper_ip, self.pepper_port)

            # eye LEDs
            session = qi.Session()
            session.connect("tcp://" + self.pepper_ip + ":" + str(self.pepper_port))
            leds = session.service("ALLeds")
            leds.setIntensity("LeftFaceLedsGreen", 1)
            leds.setIntensity("RightFaceLedsGreen", 1)
            leds.setIntensity("LeftFaceLedsBlue", 0)
            leds.setIntensity("RightFaceLedsBlue", 0)
            leds.setIntensity("LeftFaceLedsRed", 0)
            leds.setIntensity("RightFaceLedsRed", 0)


            thread = Thread(name='pepper_motion', target=self.worker)
            thread.start()

    def callback_pepper(self, joint_angle_msg):
        self.prev_motion_keys = self.motion_keys
        self.motion_keys = [joint_angle_msg.head_pitch,
                            joint_angle_msg.head_yaw,
                            joint_angle_msg.right_shoulder_pitch, 
                            joint_angle_msg.right_shoulder_roll, 
                            joint_angle_msg.right_elbow_yaw,
                            joint_angle_msg.right_elbow_roll,
                            joint_angle_msg.left_shoulder_pitch, 
                            joint_angle_msg.left_shoulder_roll, 
                            joint_angle_msg.left_elbow_yaw,
                            joint_angle_msg.left_elbow_roll]

        # for moving the robot simulation
        self.joint_state.header.stamp = rospy.Time.now();
        for idx, name in enumerate(self.motion_names):
            self.joint_state.position[self.joint_idx[name]] = self.motion_keys[idx]
        self.joint_pub.publish(self.joint_state)

    def worker(self):
        # for moving the actual robot
        while True:
            if self.prev_motion_keys == self.motion_keys:
                continue

            try:
                self.motion.angleInterpolation(self.motion_names, self.motion_keys, self.motion_times, True)
            except BaseException, err:
                self.prev_motion_keys = self.motion_keys
                print(err)

def main():
    rospy.init_node('ros_move_pepper', anonymous=True)
    only_sim = rospy.get_param("~only_sim")
    mp = MovePepper(only_sim)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()