#!/usr/bin/python3.8
from __future__ import print_function

import rospy

from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from datetime import datetime
import interface as interface

from utility import common



class XsensServer:

    def __init__(self, kwargs):


        self.interface = interface.XsensInterface(**kwargs)

        # Cartesian pose publishers
        self.all_poses_publisher = rospy.Publisher(
            "/xsens/all_poses", PoseArray, queue_size=1
        )
        self.main_body_poses_publisher = rospy.Publisher(
            "/xsens/main_body_poses", PoseArray, queue_size=1
        )
        
        self.pub_prop = kwargs["prop"]
        # Prop pose publisher
        self.prop_publisher = rospy.Publisher(
            "/xsens/prop", PoseArray, queue_size=1
        )

        self.left_finger_poses_publisher = rospy.Publisher(
            "/xsens/left_finger_poses", PoseArray, queue_size=1
        )
        self.right_finger_poses_publisher = rospy.Publisher(
            "/xsens/right_finger_poses", PoseArray, queue_size=1
        )

        self.torso_pose_publisher = rospy.Publisher(
            "/xsens/torso", PoseStamped, queue_size=1
        )
        self.left_tcp_publisher = rospy.Publisher(
            "/xsens/left_tcp", PoseStamped, queue_size=1
        )
        self.right_tcp_publisher = rospy.Publisher(
            "/xsens/right_tcp", PoseStamped, queue_size=1
        )
        self.head_publisher = rospy.Publisher("/xsens/head", PoseStamped, queue_size=1)


        self.pose_publisher_list = [
            self.all_poses_publisher,
            self.main_body_poses_publisher,
            self.prop_publisher,
            self.left_finger_poses_publisher,
            self.right_finger_poses_publisher,
            self.torso_pose_publisher,
            self.left_tcp_publisher,
            self.right_tcp_publisher,
            self.head_publisher,
        ]

        # Hand joint states publishers
        # self.left_hand_publisher = rospy.Publisher(
        #     "/xsens/left_hand_js", JointState, queue_size=1
        # )
        # self.right_hand_publisher = rospy.Publisher(
        #     "/xsens/right_hand_js", JointState, queue_size=1
        # )


        rate = kwargs["rate"]
        
        print("[Xsens] No data received")  # Overwrite the previous value
        self.all_poses_msg_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / rate), self.all_poses_msg_handle
        )

    def all_poses_msg_handle(self, event):
        pose_msg_dict = self.interface.get_datagram()
        if pose_msg_dict is None:

            print("[Xsens] No data received", end='\r')  # Overwrite the previous value
            return
        
        print("[Xsens] Data recived on ", datetime.now().strftime("%Y-%m-%d %H:%M:%S"), end='\r')
        
        cnt = 0
        for pub, msg in zip(self.pose_publisher_list, pose_msg_dict.values()):
            pub.publish(msg)
            # if cnt == 3:
            #     import numpy as np
            #     # common.plot_pose_array(np.array(msg.poses))
            #     common.save_pose_array_msg(msg, name='left_check.npy')
            # if cnt == 4:
            #     common.save_pose_array_msg(msg, name='right_check.npy')
            cnt = cnt + 1



        # left_hand_js, right_hand_js = self.interface.get_hand_joint_states()
        # if left_hand_js is not None:
        #     self.left_hand_publisher.publish(left_hand_js)
        # if right_hand_js is not None:
        #     self.right_hand_publisher.publish(right_hand_js)



if __name__ == '__main__':
    try:
        rospy.init_node("xsens_server")
        configs = {
            "udp_ip": common.get_param("ip", ""),
            "udp_port": common.get_param("port", 9763),
            "ref_frame": common.get_param("xsens_ref_frame", "world"),
            "scaling": common.get_param("scaling", 1.0),
            "rate": common.get_param("rate", 60.0),
            "prop": common.get_param("prop", False),
        }

        # If udp_ip is not given, get local IP as UDP IP
        if configs["udp_ip"] == "":
            import socket

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            configs["udp_ip"] = s.getsockname()[0]
            s.close()

        if not common.is_ip_valid(configs["udp_ip"]) or not common.is_port_valid(configs["udp_port"]):
            exit(-1)

        server = XsensServer(configs)
        rospy.loginfo("[Xsens] Xsens server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)