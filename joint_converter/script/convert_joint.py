#!/usr/bin/env python

import rospy
import tf

from visualization_msgs.msg import Marker, MarkerArray
from body_tracker_msgs.msg import Skeleton_v2
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from joint_converter_msgs.msg import PepperJoint

import torch
import torch.nn as nn
import pepper_kinematics as pk
import math
import numpy as np

from spec import JointSpec

PI = math.pi

class JointConverter(JointSpec):
    """
    Map human pose captured from a skeleton tracking software (Nuitrack)
    to pepper joint angles (map human movement to pepper movement)

    Pepper movements:
    shoulder pitch (arm swings forward)
    shoulder roll (arm swings sideways)
    elbow yaw (bicep curl)
    elbow roll (palm faces up/down)
    """
    def __init__(self, model_dir):
        super(JointConverter, self).__init__()

        # model
        model_path = model_dir + "/left_arm_ANN_model"
        self.left_arm_model = self.get_model(model_path)

        model_path = model_dir + "/right_arm_ANN_model"
        self.right_arm_model = self.get_model(model_path)

        # ros stuff
        self.skeleton_sub = rospy.Subscriber("/body_tracker/skeleton_v2", Skeleton_v2, self.callback_skeleton)
        self.pepper_joint_pub = rospy.Publisher("/pepper_joint_angles", PepperJoint, queue_size=10)
        self.visual_pub = rospy.Publisher("/pose_visualization", Marker, queue_size=10)
        self.frame_id = "nuitrack_camera_link"
        self.n_markers_prev = 0

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # joint angles
        self.prev_lshoulder_roll = 0
        self.prev_lshoulder_pitch = 0
        self.prev_lelbow_yaw = 0
        self.prev_rshoulder_roll = 0
        self.prev_rshoulder_pitch = 0
        self.prev_relbow_yaw = 0

    def get_model(self, model_path):
        model = nn.Sequential(
                    nn.Linear(6, 200),
                    nn.LeakyReLU(),
                    nn.Linear(200, 100),
                    nn.LeakyReLU(),
                    nn.Linear(100, 4)
                )
        if torch.cuda.is_available():
            state = torch.load(model_path)
        else:
            state = torch.load(model_path, map_location=torch.device('cpu'))
        model.load_state_dict(state)

        return model

    def callback_skeleton(self, skeleton_msg):
        # set origin as spine
        head = skeleton_msg.joint_position_head
        neck = skeleton_msg.joint_position_neck
        spine = skeleton_msg.joint_position_spine
        base = skeleton_msg.joint_position_base
        rhip = skeleton_msg.joint_position_right_hip
        lhip = skeleton_msg.joint_position_left_hip
        cshoulder = skeleton_msg.joint_position_shoulder_center
        rshoulder = skeleton_msg.joint_position_right_shoulder
        lshoulder = skeleton_msg.joint_position_left_shoulder
        relbow = skeleton_msg.joint_position_right_elbow
        lelbow = skeleton_msg.joint_position_left_elbow
        rwrist = skeleton_msg.joint_position_right_wrist
        lwrist = skeleton_msg.joint_position_left_wrist

        # visualize a 3d skeleton in rviz
        upper_limbs = [rwrist, relbow, rshoulder, cshoulder, lshoulder, lelbow, lwrist]
        torso = [head, neck, cshoulder, spine, base]
        hips = [rhip, base, lhip]
        self.visualize(upper_limbs, 0)
        self.visualize(torso, 1)
        self.visualize(hips, 2)

        # broadcast the points as TF
        points = [head, neck, cshoulder, spine, base, rhip, lhip, rwrist, relbow, rshoulder, lshoulder, lelbow, lwrist]
        names = ['head', 'neck', 'cshoulder', 'spine', 'base', 'rhip', 'lhip', 'rwrist', 'relbow', 'rshoulder', 'lshoulder', 'lelbow', 'lwrist']
        self.broadcast(points, names)

        # lookup the pitch angle for the head
        head_pitch = self.get_head_pitch(head, neck)

        # get desired joint positions
        desired_rwrist = self.get_desired_wrist_position(rwrist, relbow, rshoulder, right_arm=True)
        desired_lwrist = self.get_desired_wrist_position(lwrist, lelbow, lshoulder)
        desired_relbow = self.get_desired_elbow_position(relbow, rshoulder, right_arm=True)
        desired_lelbow = self.get_desired_elbow_position(lelbow, lshoulder)

        # calculate the joint angle
        # CLASSICAL APPROACH
        lshoulder_pitch, lshoulder_roll = self.get_left_shoulder_angles(lshoulder, lelbow)
        rshoulder_pitch, rshoulder_roll = self.get_right_shoulder_angles(rshoulder, relbow)
        relbow_roll = self.get_elbow_angle(rwrist, relbow, rshoulder, right_elbow=True)
        lelbow_roll = self.get_elbow_angle(lwrist, lelbow, lshoulder)
        # compare desired wrist position and actual wrist position and adjust elbow yaw accordingly <-- INCOMPLETE
        relbow_yaw = self.prev_relbow_yaw
        lelbow_yaw = self.prev_lelbow_yaw
        actual_rwrist = self.get_actual_position(rshoulder_pitch, rshoulder_roll, relbow_yaw, relbow_roll, right_arm=True)
        actual_lwrist = self.get_actual_position(lshoulder_pitch, lshoulder_roll, lelbow_yaw, lelbow_roll)

        # AI APPROACH (use only when arm is not outstretched to the sides)
        if not (desired_lelbow.position.x - 0.05 <= desired_lwrist.position.x <= desired_lelbow.position.x + 0.05 and 
                desired_lwrist.position.y > desired_lelbow.position.y + 0.15):
            model_input = torch.Tensor([desired_lwrist.position.x, desired_lwrist.position.y, desired_lwrist.position.z,
                                        desired_lelbow.position.x, desired_lelbow.position.y, desired_lelbow.position.z])
            model_output = self.left_arm_model(model_input)
            model_output = model_output.detach().numpy()
            for idx, angle in enumerate(model_output):
                lower, upper = self.left_arm_range[idx]
                if angle < lower:
                    model_output[idx] = lower
                elif angle > upper:
                    model_output[idx] = upper
            lshoulder_pitch, lshoulder_roll, lelbow_yaw, lelbow_roll = model_output

        if not (desired_relbow.position.x - 0.05 <= desired_rwrist.position.x <= desired_relbow.position.x + 0.05 and 
                desired_rwrist.position.y < desired_relbow.position.y - 0.15):
            model_input = torch.Tensor([desired_rwrist.position.x, desired_rwrist.position.y, desired_rwrist.position.z,
                                        desired_relbow.position.x, desired_relbow.position.y, desired_relbow.position.z])
            model_output = self.right_arm_model(model_input)
            model_output = model_output.detach().numpy()
            for idx, angle in enumerate(model_output):
                lower, upper = self.right_arm_range[idx]
                if angle < lower:
                    model_output[idx] = lower
                elif angle > upper:
                    model_output[idx] = upper
            rshoulder_pitch, rshoulder_roll, relbow_yaw, relbow_roll = model_output

        # publish the pepper joints
        joints = PepperJoint()
        joints.head_pitch = head_pitch
        joints.head_yaw = 0
        joints.right_shoulder_pitch = rshoulder_pitch
        joints.right_shoulder_roll = rshoulder_roll
        joints.right_elbow_yaw = relbow_yaw
        joints.right_elbow_roll = relbow_roll
        joints.left_shoulder_pitch = lshoulder_pitch
        joints.left_shoulder_roll = lshoulder_roll
        joints.left_elbow_yaw = lelbow_yaw
        joints.left_elbow_roll = lelbow_roll
        self.pepper_joint_pub.publish(joints)

    def visualize(self, points, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0 # make it transparent
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = [point.position for point in points]
        self.visual_pub.publish(marker)

    def visualize_desired_position(self, position, marker_id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0 # make it transparent
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose = position
        self.visual_pub.publish(marker)

    def broadcast(self, points, frame_names):
        for idx, point in enumerate(points):
            pos = point.position
            quat = point.orientation
            self.br.sendTransform(  (pos.x, pos.y, pos.z), 
                                    (quat.x, quat.y, quat.z, quat.w), 
                                    rospy.Time(0), 
                                    frame_names[idx], # new TF frame name
                                    self.frame_id) # original frame id reference

    def get_diff(self, base, target):
        # +x left
        # -x right
        # +z away from camera
        # -z toward camera
        # +y up
        # -y down
        delta_x = base.position.x - target.position.x
        delta_y = base.position.y - target.position.y
        delta_z = base.position.z - target.position.z

        return delta_x, delta_y, delta_z

    def convert_frame(self, target, base, ratio):
        """
        convert the target frame with respect to the base frame 
        and scale based on the ratio

        NOTE:
            For the camera frame:
                +x right (pov as camera)
                -x left 
                +z away from camera
                -z toward camera
                +y up
                -y down

            For the pepper base frame
                +x forward
                -x backward
                +y left (pov as pepper)
                -y right
                +z up
                -z down
        """
        x = -(target.position.z - base.position.z) * ratio
        y = (target.position.x - base.position.x) * ratio
        z = (target.position.y - base.position.y) * ratio
        target.position.x = x
        target.position.y = y
        target.position.z = z
        return target

    def get_dist(self, p_1, p_2):
        p1 = [p_1.position.x, p_1.position.y, p_1.position.z]
        p2 = [p_2.position.x, p_2.position.y, p_2.position.z] if type(p_2) != list else p_2 
        d = np.array(p1) - np.array(p2)
        return np.sqrt(np.dot(d.T,d))

    def get_head_pitch(self, head, neck):
        _, y, z = self.get_diff(head, neck)
        head_pitch = math.atan(y / z) if abs(z) > 0.01 else 0
        upper, lower = 0.64, -0.71
        if head_pitch > 0:
            # head up
            head_pitch -= PI/2 # range: -0.2 to -0.55
            head_pitch *= 1.3
            head_pitch = lower if head_pitch < lower else head_pitch
        elif head_pitch < 0:
            # head down
            head_pitch += PI/2 # range: 0.17 1.0
            head_pitch *= 0.64
            head_pitch = upper  if head_pitch > upper else head_pitch

        return head_pitch

    def get_left_shoulder_angles(self, lshoulder, lelbow):
        """
        return the pitch and roll angle of the left shoulder
        """
        x, y, z = self.get_diff(lshoulder, lelbow)
        den = math.sqrt(z**2 + x**2)
        lower, upper = self.left_arm_range_dict["LShoulderRoll"]
        lshoulder_roll = upper - (-math.atan(z/x)) if x else 0 
        if lshoulder_roll > upper: 
            # when joint flipping occurs, the joint values are always greater than the upper range
            # to determine whether to set the joint value to upper or lower range, refer to prev joint value
            lshoulder_roll = upper if self.prev_lshoulder_roll > upper - 0.5 else lower
        self.prev_lshoulder_roll = lshoulder_roll
        lshoulder_pitch = math.atan(y/den) if den else 0
        lower, upper = self.left_arm_range_dict["LShoulderPitch"]
        if lshoulder_pitch > upper:
            lshoulder_pitch = upper if self.prev_lshoulder_pitch > upper - 0.5 else lower
        self.prev_lshoulder_pitch = lshoulder_pitch

        return lshoulder_pitch, lshoulder_roll

    def get_right_shoulder_angles(self, rshoulder, relbow):
        """
        return the pitch and roll angle of the right shoulder
        """
        x, y, z = self.get_diff(rshoulder, relbow)
        den = math.sqrt(z**2 + x**2)
        lower, upper = self.right_arm_range_dict["RShoulderRoll"]
        rshoulder_roll = lower - (-math.atan(z/x)) if x else 0 
        if rshoulder_roll < lower: 
            # when joint flipping occurs, the joint values are always less than lower range
            # to determine whether to set the joint value to upper or lower range, refer to prev joint value
            rshoulder_roll = upper if self.prev_rshoulder_roll > upper - 0.5 else lower
        self.prev_rshoulder_roll = rshoulder_roll
        rshoulder_pitch = math.atan(y/den) if den else 0
        lower, upper = self.right_arm_range_dict["RShoulderPitch"]
        if rshoulder_pitch < lower:
            rshoulder_pitch = upper if self.prev_rshoulder_pitch > upper - 0.5 else lower
        self.prev_rshoulder_pitch = rshoulder_pitch

        return rshoulder_pitch, rshoulder_roll

    def get_elbow_angle(self, A, B, C, right_elbow=False):
        # courtesy of Jeff's Baxter Code
        # ranges from 3 (straight arm) to 1.5 (90 degree bend elbow)
        AB = self.get_dist(A,B)
        BC = self.get_dist(B,C)
        AC = self.get_dist(A,C)

        try:
            ABC_angle = math.acos((AB**2 + BC**2 - AC**2)/(2*AB*BC))
        except ValueError as e:
            if "math domain error" in str(e):
                ABC_angle = 0
            else:
                raise e

        # convert to pepper range, from 0 (straight arm) to -1.56 or 1.56 (90 degree bend elbow)
        ABC_angle = max(ABC_angle, 1.5)     # joint limit
        ABC_angle = ABC_angle - 3.0         # reset the upper range to 0
        ABC_angle = ABC_angle * (PI/2)/1.5  # multiply by the scaling ratio
        if right_elbow:
            ABC_angle = ABC_angle * -1

        return ABC_angle

    def get_desired_elbow_position(self, elbow, shoulder, right_arm=False):
        """
        get desired position of Pepper's elbows

        returns a Pose msg
        """
        # convert elbow joint frames to Pepper perspective (scale appropriately, change x y z axis)
        human_elbow_length = self.get_dist(elbow, shoulder)
        elbow_ratio = self.pepper_elbow_length / human_elbow_length
        desired_elbow = self.convert_frame(elbow, shoulder, elbow_ratio)
        # set the joint frames with respect to Pepper base frame (which is near the spine)
        desired_elbow.position.z += self.pepper_shoulder_to_base_link_z
        desired_elbow.position.x += self.pepper_shoulder_to_base_link_x
        if right_arm:
            desired_elbow.position.y = desired_elbow.position.y - self.pepper_shoulder_width / 2
            marker_id = 5
        else:
            desired_elbow.position.y = desired_elbow.position.y + self.pepper_shoulder_width / 2
            marker_id = 6
        self.visualize_desired_position(desired_elbow, marker_id)
        return desired_elbow

    def get_desired_wrist_position(self, wrist, elbow, shoulder, right_arm=False):
        """
        get desired position of Pepper's hands

        returns a Pose msg
        """
        # convert wrist joint frames to Pepper perspective (scale appropriately, change x y z axis)
        human_arm_length = self.get_dist(wrist, elbow) + self.get_dist(elbow, shoulder)
        arm_ratio = self.pepper_arm_length / human_arm_length
        desired_wrist = self.convert_frame(wrist, shoulder, arm_ratio)
        # set the joint frames with respect to Pepper base frame (which is near the spine)
        desired_wrist.position.z += self.pepper_shoulder_to_base_link_z
        desired_wrist.position.x += self.pepper_shoulder_to_base_link_x
        if right_arm:
            desired_wrist.position.y = desired_wrist.position.y - self.pepper_shoulder_width / 2
            marker_id = 3
        else:
            desired_wrist.position.y = desired_wrist.position.y + self.pepper_shoulder_width / 2
            marker_id = 4
        self.visualize_desired_position(desired_wrist, marker_id)
        return desired_wrist

    def get_actual_position(self, shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, right_arm=False):
        """
        get actual position of Pepper's hands

        returns a list [x, y, z]
        """
        if right_arm:
            actual_wrist, _ = pk.right_arm_get_position([shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, 0])
        else:
            actual_wrist, _ = pk.left_arm_get_position([shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, 0])
        actual_wrist[2] += self.pepper_head_to_base_link_z # pk outputs position with respect to head frame, not base_link frame
        actual_wrist[0] += self.pepper_head_to_base_link_x # pk outputs position with respect to head frame, not base_link frame
        actual_wrist = actual_wrist[:-1] # pepper_kinematics has an extra 1 at the end that's not needed
        return list(actual_wrist)

def main():
    rospy.init_node('ros_joint_converter', anonymous=True, disable_signals=True)
    model_dir = rospy.get_param("~model_dir")
    jc = JointConverter(model_dir)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()