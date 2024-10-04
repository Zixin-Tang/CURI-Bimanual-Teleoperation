#!/usr/bin/python3.8
import rospy
import numpy as np

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from datetime import datetime

from utility import common
from utility.safe_checker import SafeChecker



class SoftHand2Wrapper(SafeChecker):
    def __init__(self, kwargs):
        super().__init__(kwargs["enable_safechecker"], kwargs["curi_audio_srv_name"])
        
        self.finger_joint_names = [
            "thumb1",
            "thumb2",
            "thumb3",
            "index1",
            "index2",
            "index3",
            "middle1",
            "middle2",
            "middle3",
            "ring1",
            "ring2",
            "ring3",
            "pinky1",
            "pinky2",
            "pinky3",
        ]

        self.synergy2joint_matrix = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], 
                                              [2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2]], dtype=np.float32)
        
        self.joint2synergy_matrix = np.linalg.pinv(self.synergy2joint_matrix)
        
        self.min_manus_joint = 0
        self.max_manus_joint = 1.57

        self.min_softhand_joints = np.zeros(15)  # [0, 0]
        self.max_softhand_joints = np.array([2.5, 2.5, 2.5, 1.5, 1.5, 1.5,0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -1.5, -1.5, -1.5])  # [1, 1]


        left_finger_poses_xsens_topic = kwargs["left_finger_poses_xsens_topic"]
        right_finger_poses_xsens_topic = kwargs["right_finger_poses_xsens_topic"]
        left_finger_joints_topic = kwargs["left_finger_joints_topic"]
        right_finger_joints_topic = kwargs["right_finger_joints_topic"]
        left_softhand_synergies_topic = kwargs["left_softhand_synergies_topic"]
        right_softhand_synergies_topic = kwargs["right_softhand_synergies_topic"]

        left_sub = rospy.Subscriber(left_finger_poses_xsens_topic, PoseArray, self.retargeting, (1))
        right_sub = rospy.Subscriber(right_finger_poses_xsens_topic, PoseArray, self.retargeting, (2))

        left_joint_pub = rospy.Publisher(left_finger_joints_topic, JointState, queue_size=1)
        right_joint_pub = rospy.Publisher(right_finger_joints_topic, JointState, queue_size=1)
        left_synergy_pub = rospy.Publisher(left_softhand_synergies_topic, JointTrajectory, queue_size=1)
        right_synergy_pub = rospy.Publisher(right_softhand_synergies_topic, JointTrajectory, queue_size=1)
        
        self.joint_pub_list = [left_joint_pub, right_joint_pub]
        self.synergy_pub_list = [left_synergy_pub, right_synergy_pub]


    def retargeting(self, msg, device_id):

        # mapping finger poses from xsens to finger joints of manus, and publishing joints to relevant topics
        each_hand_finger_joints = self.extract_joints(msg, device_id)

        # mapping finger joints of manus to two softhand synergies, and publishing synergies to relevant topics
        self.mapping_to_synergies(each_hand_finger_joints, device_id)
        

    def extract_joints(self, msg, device_id):
        """
        - msg: PoseArray
        """

        # each item of poses is a PoseStamped
        poses = msg.poses

        non_thumb_finger_joints = []

        for non_thumb_ind in [5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19]:
            # get x from x-y-z euler
            relative_pose = common.get_transform_same_base(poses[non_thumb_ind-1], poses[non_thumb_ind])
            euler = common.transform_to_euler(relative_pose)
            # if non_thumb_ind == 5:
            #     print(euler)
            non_thumb_finger_joints.append(abs(euler[0]))

        
        thumb_finger_joints = []
        thumb_metacarpus_position = common.sd_pose(poses[1])[:3, 3] 
        thumb1_joint = abs(np.arcsin(thumb_metacarpus_position[2] / np.linalg.norm(thumb_metacarpus_position)))
        # print(np.linalg.norm(thumb_metacarpus_position))
        # print(thumb1_joint)
        # print(thumb_metacarpus_position)
        thumb_finger_joints.append(thumb1_joint)
        for thumb_ind in [2, 3]:
            # get z from x-y-z euler
            relative_pose = common.get_transform_same_base(poses[thumb_ind-1], poses[thumb_ind])
            euler = common.transform_to_euler(relative_pose)
            thumb_finger_joints.append(abs(euler[2]))
        # print(non_thumb_finger_joints[:3])

        all_finger_joints = np.array(thumb_finger_joints + non_thumb_finger_joints)
        all_finger_joints = np.clip(all_finger_joints, self.min_manus_joint, self.max_manus_joint)
        
        assert len(all_finger_joints) == 15, "the number of all finger joints each hand should be 15"

        joint_msg = common.to_ros_jointstate(self.finger_joint_names, all_finger_joints, [0.0] * len(all_finger_joints), [0.0] * len(all_finger_joints))

        self.joint_pub_list[device_id-1].publish(joint_msg)

        return all_finger_joints


    def mapping_to_synergies(self, all_finger_joints, device_id, secs_to_perform=1):

        scaled_joint = (all_finger_joints - self.min_manus_joint) / self.max_manus_joint * (self.max_softhand_joints - self.min_softhand_joints) + self.min_softhand_joints
        # print(scaled_joint)
        synergies = np.dot(scaled_joint, self.joint2synergy_matrix)
        
        manipulation_joint = synergies[0] * 1.5
        # manipulation_joint = min(max(-1, manipulation_joint), 1)
        synergy_joint = synergies[1] * 1.5
        # synergy_joint = min(max(synergy_joint, 0), 1)

        synergy_msg = common.to_ros_jointtrajectory(joint_name_list=["qbhand2m%d_manipulation_joint" % device_id, "qbhand2m%d_synergy_joint" % device_id],
                                                    joint_pos_list=[manipulation_joint, synergy_joint],
                                                    joint_vel_list=[0.0, 0.0], joint_acc_list=[0.0, 0.0], joint_effort_list=[0.0, 0.0], secs_to_perform=secs_to_perform)

        self.synergy_pub_list[device_id-1].publish(synergy_msg)

        print("[Softhand] Publishing synergies command on ", datetime.now().strftime("%Y-%m-%d %H:%M:%S"), end='\r')  # Overwrite the previous value
        
        # rospy.signal_shutdown('Event received, shutting down node.')





if __name__ == "__main__":
    # common.plot_pose_array_from_file("/home/clover/Desktop/left_check.npy")
    try:
        rospy.init_node("teleop_softhand")        
        configs = {
            "enable_safechecker": True,
            "curi_audio_srv_name": "/teleoperation/audio", 
            "left_finger_poses_xsens_topic": common.get_param("left_finger_poses_xsens_topic"),
            "right_finger_poses_xsens_topic": common.get_param("right_finger_poses_xsens_topic"),
            "left_finger_joints_topic": common.get_param("left_finger_joints_topic"),
            "right_finger_joints_topic": common.get_param("right_finger_joints_topic"),
            "left_softhand_synergies_topic": common.get_param("left_softhand_synergies_topic"),
            "right_softhand_synergies_topic": common.get_param("right_softhand_synergies_topic")
        }


        SoftHand2Wrapper(configs)
        rospy.loginfo("[EETeleop]: Softhand server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)