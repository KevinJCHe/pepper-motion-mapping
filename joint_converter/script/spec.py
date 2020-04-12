#!/usr/bin/env python
from sensor_msgs.msg import JointState

class JointSpec(object):
    """
    peppper specificiations: 
        - http://doc.aldebaran.com/2-4/family/pepper_technical/links_pep.html
        - http://doc.aldebaran.com/2-4/family/pepper_technical/joints_pep.html
    
    NOTE: Pepper's base frame is located at the bottom of Pepper's tablet
    """
    def __init__(self):
        self.right_arm_tags = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        self.right_arm_range = [(-2.0857, 2.0857), (-1.5620, -0.0087), (-2.0857, 2.0857), (0.0087, 1.5620), (-1.8239, 1.8239)] # rad
        self.right_arm_range_dict = {tag: self.right_arm_range[idx] for idx, tag in enumerate(self.right_arm_tags)}
        self.left_arm_tags = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        self.left_arm_range = [(-2.0857, 2.0857), (0.0087, 1.5620), (-2.0857, 2.0857), (-1.5620, -0.0087), (-1.8239, 1.8239)] # rad
        self.left_arm_range_dict = {tag: self.left_arm_range[idx] for idx, tag in enumerate(self.left_arm_tags)}
        self.head_pitch_range = (0.6371, -0.7068) # rad

        self.pepper_arm_length = 0.3312 # m
        self.pepper_elbow_length = 0.1812 # m
        self.pepper_shoulder_width = 0.29948 # m
        self.pepper_shoulder_to_base_link_z = 0.1 # m
        self.pepper_shoulder_to_base_link_x = -0.05 # m
        self.pepper_head_to_base_link_z = 0.1699 # m
        self.pepper_head_to_base_link_x = -0.04 # m