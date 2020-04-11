#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState
from spec import joint_spec
import pandas as pd
import numpy as np
import math
import time

PI = math.pi
data_points = 2500000
batch_size = 10000

class DataCollector(JointSpec):
    """
    Collects training data for the model
        - input: joint angles for shoulder pitch, shoulder roll, elbow yaw, and elbow roll for both arms
        - output: the x y z coordinates of the wrist and elbow joints 
    """
    def __init__(self):
        super(DataCollector, self).__init__()
        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10, latch=True)
        self.listener = tf.TransformListener()
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

        # self.get_data_points() # for left arm
        # self.get_data_points(right_arm=True) # for right arm
        self.check_data()

    def get_data_points(self, right_arm=False):
        X = "R" if right_arm else "L"
        x = "r" if right_arm else "l"
        arm_range_dict = self.right_arm_range_dict if right_arm else self.left_arm_range_dict
        arm_tags = self.right_arm_tags if right_arm else self.left_arm_tags
        file_name = "right" if right_arm else "left"
        columns = arm_tags + ["{}wrist_x".format(x), "{}wrist_y".format(x), "{}wrist_z".format(x), "{}elbow_x".format(x), "{}elbow_y".format(x), "{}elbow_z".format(x)]

        data = []
        for count in range(1, data_points+1):
            # grab random values for the joint angles
            lower, upper = arm_range_dict['{}ShoulderPitch'.format(X)]
            shoulder_pitch = np.random.uniform(lower, upper)
            lower, upper = arm_range_dict['{}ShoulderRoll'.format(X)]
            shoulder_roll = np.random.uniform(lower, upper)
            lower, upper = arm_range_dict['{}ElbowYaw'.format(X)]
            elbow_yaw = np.random.uniform(lower, upper)
            lower, upper = arm_range_dict['{}ElbowRoll'.format(X)]
            elbow_roll = np.random.uniform(lower, upper)

            # publish the angles to the pepper joint states
            self.joint_state.header.stamp = rospy.Time.now();
            self.joint_state.position[self.joint_idx['{}ShoulderPitch'.format(X)]] = shoulder_pitch
            self.joint_state.position[self.joint_idx['{}ShoulderRoll'.format(X)]] = shoulder_roll
            self.joint_state.position[self.joint_idx['{}ElbowYaw'.format(X)]] = elbow_yaw
            self.joint_state.position[self.joint_idx['{}ElbowRoll'.format(X)]] = elbow_roll
            self.joint_pub.publish(self.joint_state)
            time.sleep(0.1) # allow some time for the joint states to be published

            # grab the wrist and elbow position
            self.listener.waitForTransform('/base_link','/{}Elbow'.format(X),rospy.Time(), rospy.Duration(4.0))
            elbow, _ = self.listener.lookupTransform('/base_link','/{}Elbow'.format(X),rospy.Time(0))
            self.listener.waitForTransform('/base_link','/{}_wrist'.format(x),rospy.Time(), rospy.Duration(4.0))
            wrist, _ = self.listener.lookupTransform('/base_link','/{}_wrist'.format(x),rospy.Time(0))
            
            data.append([shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, 
                         wrist[0], wrist[1], wrist[2], elbow[0], elbow[1], elbow[2]])

            # batch write every 10000 data points
            if count % batch_size == 0:
                df = pd.DataFrame(data, columns=columns)
                data = []
                print("Writing {} data points to csv file".format(count))
                if count == batch_size:
                    df.to_csv('/home/kevinh/nuitrack_ws/src/pepper-motion-mapping/joint_converter/script/{}_arm_data.csv'.format(file_name), index=False)
                else:
                    df.to_csv('/home/kevinh/nuitrack_ws/src/pepper-motion-mapping/joint_converter/script/{}_arm_data.csv'.format(file_name), index=False, mode='a', header=False)

    def check_data(self):
        df = pd.read_csv("/home/kevinh/Documents/right_arm_data.csv")
        X = "R"
        x = "r"
        angle_names = ["{}ShoulderPitch".format(X),
                       "{}ShoulderRoll".format(X),
                       "{}ElbowYaw".format(X),
                       "{}ElbowRoll".format(X)]
        labels = df[angle_names]
        data = df[[column for column in df.columns if column not in angle_names]]
        print("unfiltered data points",len(data))
        # filter corrupted data!!!
        data = data.drop_duplicates(keep=False)
        labels = labels.loc[data.index]
        print("filtered data points",len(data))
        for i in range(len(data)):
            shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll = labels.iloc[i].values
            # publish the angles to the pepper joint states
            self.joint_state.header.stamp = rospy.Time.now();
            self.joint_state.position[self.joint_idx['{}ShoulderPitch'.format(X)]] = shoulder_pitch
            self.joint_state.position[self.joint_idx['{}ShoulderRoll'.format(X)]] = shoulder_roll
            self.joint_state.position[self.joint_idx['{}ElbowYaw'.format(X)]] = elbow_yaw
            self.joint_state.position[self.joint_idx['{}ElbowRoll'.format(X)]] = elbow_roll
            self.joint_pub.publish(self.joint_state)
            time.sleep(0.1) # allow some time for the joint states to be published

            # grab the wrist and elbow position
            self.listener.waitForTransform('/base_link','/{}Elbow'.format(X),rospy.Time(), rospy.Duration(4.0))
            elbow, _ = self.listener.lookupTransform('/base_link','/{}Elbow'.format(X),rospy.Time(0))
            self.listener.waitForTransform('/base_link','/{}_wrist'.format(x),rospy.Time(), rospy.Duration(4.0))
            wrist, _ = self.listener.lookupTransform('/base_link','/{}_wrist'.format(x),rospy.Time(0))

            a = list(np.round(data.iloc[i].values, decimals=4))
            b = list(np.round(wrist, decimals=4)) + list(np.round(elbow, decimals=4))
            if not all(element == b[idx] for idx, element in enumerate(a)):
                print(a)
                print(b)
                print("---------------------------------------------")

def main():
    rospy.init_node('ros_data_collector', anonymous=True)
    dc = DataCollector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()