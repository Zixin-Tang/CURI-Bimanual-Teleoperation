#!/usr/bin/python3.8
import rospy
import numpy as np
import keyboard

from datetime import datetime

from utility import common


import rospy
import math
from geometry_msgs.msg import PoseStamped

from utility import common
from tf_checker import TFChecker
from utility.safe_checker import SafeChecker



class FrankaWrapper(SafeChecker):
    def __init__(self, kwargs):
        super().__init__(kwargs["enable_safechecker"], kwargs["curi_audio_srv_name"])
        
        self.xsens_base_frame = kwargs["xsens_base_frame"]
        self.ee_type = kwargs["ee_type"]

        left_arm_pose_xsens_topic = kwargs["left_arm_pose_xsens_topic"]
        right_arm_pose_xsens_topic = kwargs["right_arm_pose_xsens_topic"]

        left_arm_control_topic = kwargs["left_arm_control_topic"]
        right_arm_control_topic = kwargs["right_arm_control_topic"]
    

        left_sub = rospy.Subscriber(left_arm_pose_xsens_topic, PoseStamped, self.retargeting, (1))
        right_sub = rospy.Subscriber(right_arm_pose_xsens_topic, PoseStamped, self.retargeting, (2))

        left_control_pub = rospy.Publisher(left_arm_control_topic, PoseStamped, queue_size=1)
        right_control_pub = rospy.Publisher(right_arm_control_topic, PoseStamped, queue_size=1)
        
        self.control_pub_list = [
            left_control_pub,
            right_control_pub
        ]


        left_tf_checker = TFChecker(parent_frame="panda_left_link0", child_frame="retargeted_left_lin8")
        right_tf_checker = TFChecker(parent_frame="panda_right_link0", child_frame="retargeted_right_lin8")
        self.tf_checker_list = [
            left_tf_checker,
            right_tf_checker
        ]


        if self.ee_type == "gripper":
            left_post_multiply_R = common.euler_to_rotation_matrix([-math.pi/2, 0, -math.pi/2], order="XYZ", degrees=False)
            right_post_multiply_R = common.euler_to_rotation_matrix([math.pi/2, 0, math.pi/2], order="XYZ", degrees=False)
        elif self.ee_type == "softhand":
            left_post_multiply_R = common.euler_to_rotation_matrix([-math.pi, 0, -math.pi/2], order="XYZ", degrees=False)
            right_post_multiply_R = common.euler_to_rotation_matrix([math.pi, 0, math.pi], order="XYZ", degrees=False)

        else:
            rospy.logwarn("EE type %s is not supported, using default type 'gripper'" % self.ee_type)
            left_post_multiply_R = common.euler_to_rotation_matrix([-math.pi/2, 0, -math.pi/2], order="XYZ", degrees=False)
            right_post_multiply_R = common.euler_to_rotation_matrix([math.pi/2, 0, math.pi/2], order="XYZ", degrees=False)

        self.delta_pose_list = [
            common.rotation_matrix_to_transformation_matrix(left_post_multiply_R),
            common.rotation_matrix_to_transformation_matrix(right_post_multiply_R)
        ]

        
    def retargeting(self, msg, device_id):
        
        xsens_ee_pose = common.sd_pose(msg)
        delta_pose = self.delta_pose_list[device_id-1]

        franka_tcp_pose = np.dot(xsens_ee_pose, delta_pose)
        # franka_tcp_pose = np.dot(delta_pose, xsens_ee_pose)

        # scaling translation part
        franka_tcp_pose[:3, 3] = franka_tcp_pose[:3, 3] * np.array([2, 3, 2]) + np.array([0, 0, 0.2])

        if self.xsens_base_frame == "t8":
            if device_id == 1:
                # left arm
                franka_tcp2armbase_pose = np.dot(np.linalg.inv(self.left_armbase2torsoend), franka_tcp_pose)
                # print("left\n:", franka_tcp_pose)
                # print("left pos\n:", common.to_list(franka_tcp2armbase_pose[:3, 3]))
                # print("left quat\n:", common.to_list(common.rotation_matrix_to_quat(franka_tcp2armbase_pose[:3, :3])))

            elif device_id == 2:
                # right arm
                franka_tcp2armbase_pose = np.dot(np.linalg.inv(self.right_armbase2torsoend), franka_tcp_pose)
            else:
                rospy.logerr("Device_id should be 1 for left or 2 for right, not supporting %d" % device_id)

            pose_stamped_msg = common.to_ros_pose_stamped(franka_tcp2armbase_pose)

            self.tf_checker_list[device_id-1].send_tf(pose_stamped_msg.pose)
            # print(self.send, "///")
            # if self.send and device_id==1:
            if self.send:
                self.control_pub_list[device_id-1].publish(pose_stamped_msg)
                print("[Franka] Publishing control command on ", datetime.now().strftime("%Y-%m-%d %H:%M:%S"), end='\r')  # Overwrite the previous value
            else:
                pass
                # print("[Franka] Please say 'start' to start arm teleoperation", end='\r')
            
        else:
            rospy.logerr("Poses from xsens should based in 't8' frame")

    @property
    def left_armbase2torsoend(self):
        """
            Fixed transformation matrix from armbase to torsoend (base frame)
            return (4, 4) matrix
        """
        T_left_torsoend2armbase = self.transform_torsoend_to_armbase(
            np.array([math.pi / 2, -math.pi / 4, -math.pi / 6, -math.pi / 18]),
            np.array([[-0.08537, 0.07009, 0.2535]]),
        )
        
        return T_left_torsoend2armbase

    @property
    def right_armbase2torsoend(self):
        """
            Fixed transformation matrix from armbase to torsoend (base frame)
            return (4, 4) matrix
        """
        T_right_torsoend2armbase = self.transform_torsoend_to_armbase(
            np.array([math.pi / 2, math.pi / 4, math.pi / 6, -math.pi / 18]),
            np.array([[-0.08537, -0.07009, 0.2535]]),
        )
        return T_right_torsoend2armbase




    def transform_torsoend_to_armbase(self, data, vec):
        R_1 = np.array(
            [
                [math.cos(data[0]), 0, math.sin(data[0])],
                [0, 1, 0],
                [-math.sin(data[0]), 0, math.cos(data[0])],
            ]
        )
        R_2 = np.array(
            [
                [1, 0, 0],
                [0, math.cos(data[1]), -math.sin(data[1])],
                [0, math.sin(data[1]), math.cos(data[1])],
            ]
        )
        R_3 = np.array(
            [
                [math.cos(data[2]), -math.sin(data[2]), 0],
                [math.sin(data[2]), math.cos(data[2]), 0],
                [0, 0, 1],
            ]
        )
        R_4 = np.array(
            [
                [math.cos(data[3]), 0, math.sin(data[3])],
                [0, 1, 0],
                [-math.sin(data[3]), 0, math.cos(data[3])],
            ]
        )
        T = np.around(np.array([R_4 @ R_1 @ R_2 @ R_3]).reshape(-1, 3), decimals=6)
        T = np.r_[np.c_[T, vec.T], np.array([[0, 0, 0, 1]])]

        return T
        
if __name__ == "__main__":

    try:
        rospy.init_node("teleop_franka")

        configs = {
            "enable_safechecker": True,
            "curi_audio_srv_name": "/teleoperation/audio", 
            "left_arm_pose_xsens_topic": common.get_param("left_arm_pose_xsens_topic"),
            "right_arm_pose_xsens_topic": common.get_param("right_arm_pose_xsens_topic"),
            
            "xsens_base_frame": common.get_param("xsens_ref_frame", "t8").lower(),  # defined in run_xsens.launch
            "ee_type": "softhand", # common.get_param("ee_type", "gripper"),  # defined in run_eeteleop.launch

            "left_arm_control_topic": common.get_param("left_arm_control_topic"),
            "right_arm_control_topic": common.get_param("right_arm_control_topic")
        }

        wrapper = FrankaWrapper(configs)


        rospy.loginfo("[ArmTeleop]: Franka server (equipped with %s) ready." % configs["ee_type"])
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
