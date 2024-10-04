from __future__ import print_function

import socket
import sys
import numpy as np

try:
    import rospy
    import tf2_ros
    import tf2_geometry_msgs  # import this is mandatory to use PoseStamped msg

    import moveit_commander

    import geometry_msgs.msg as geo_msg
    import sensor_msgs.msg as sensor_msg
except ImportError:
    rospy = None
    geo_msg = None
    sensor_msg = None

# import rospkg
# sys.path.append(rospkg.RosPack().get_path('utility'))
from utility import common


class Header:
    def __init__(self, header):
        assert isinstance(header, list) and len(header) == 10
        self.ID_str = header[0]
        self.sample_counter = header[1]
        self.datagram_counter = header[2]
        self.item_counter = header[3]  # Amount of items (point/segments) body+prop+finger
        self.time_code = header[4]  # Time since start measurement
        self.character_ID = header[5]  # Amount of people recorded at the same time
        self.body_segments_num = header[6]  # number of body segments measured
        self.props_num = header[7]  # Amount of property sensors
        self.finger_segments_num = header[8]  # Number of finger data segments
        self.payload_size = header[9]  # Size of the measurement excluding the header

    def __repr__(self):
        s = (
            "Header {}: \nsample_counter {}, datagram_counter {},\n"
            "item #{}, body segment #{}, prop #{}, finger segment #{}\n".format(
                self.ID_str,
                self.sample_counter,
                self.datagram_counter,
                self.item_counter,
                self.body_segments_num,
                self.props_num,
                self.finger_segments_num,
            )
        )
        return s

    @property
    def is_valid(self):
        if self.ID_str != "MXTP02":
            rospy.logwarn(
                "XSensInterface: Currently only support MXTP02, but got {}".format(
                    self.ID_str
                )
            )
            return False
        if (
            self.item_counter
            != self.body_segments_num + self.props_num + self.finger_segments_num
        ):
            rospy.logwarn(
                "XSensInterface: Segments number in total does not match item counter"
            )
            return False
        if self.payload_size % self.item_counter != 0:
            rospy.logwarn(
                "XSensInterface: Payload size {} is not dividable by item number {}".format(
                    self.payload_size, self.item_num
                )
            )
            return False
        return True

    


class Datagram(object):
    def __init__(self, header, payload):
        self.header = header
        self.payload = payload

    def parsing(
        self, ref_frame, ref_frame_id=None, scaling_factor=1.0
    ):
        """Decode the bytes in the streaming data to pose array message.

        1. The body poses excepts hand segment poses
        2. Prop segment poses if existed
        2. Finger segment poses if existed
        3. Interested key poses.

        :param ref_frame: str Reference frame name of the generated pose array message.
        :param ref_frame_id: None/int If not None, all poses will be shifted subject to
                             the frame with this ID. This frame should belong to the human.
        :param scaling_factor: float Scale the position of the pose if src_frame_id is not None.
                               Its value equals to the robot/human body dimension ratio
        :return: PoseArray PoseStamped ...

        """
        all_pose_list = []

        for i in range(self.item_num):
            item = self.payload[
                i * self.item_size : (i + 1) * self.item_size
            ]
            pose = self._decode_to_pose(item)
            if pose is None:
                return None
            all_pose_list.append(pose)

        
        # all_poses should at least contain body segment poses
        assert len(all_pose_list) == len(self.main_body_segment_index) + len(self.prop_segment_index) + len(self.finger_segment_index), \
            rospy.logerr(
            "XSensInterface: all_segments_poses should at least contain body segment poses"
        )

        _stamp = rospy.Time.now()

        # all msg
        all_pose_msg = geo_msg.PoseArray()
        all_pose_msg.header.stamp = _stamp
        all_pose_msg.header.frame_id = ref_frame

        # main body
        main_body_msg = geo_msg.PoseArray()
        main_body_msg.header = all_pose_msg.header

        # prop
        prop_msg = geo_msg.PoseArray()
        prop_msg.header = main_body_msg.header

        # left finger
        left_finger_msg = geo_msg.PoseArray()
        left_finger_msg.header.stamp = _stamp
        left_finger_msg.header.frame_id = "left_carpus"
            
        # right finger
        right_finger_msg = geo_msg.PoseArray()
        right_finger_msg.header.stamp = _stamp
        right_finger_msg.header.frame_id = "right_carpus"
        
        ## interested key poses
        torso_pose_msg = geo_msg.PoseStamped()
        left_tcp_msg = geo_msg.PoseStamped()
        right_tcp_msg = geo_msg.PoseStamped()
        head_msg = geo_msg.PoseStamped()

        # Initialize message headers
        torso_pose_msg.header = main_body_msg.header
        left_tcp_msg.header = main_body_msg.header
        right_tcp_msg.header = main_body_msg.header
        head_msg.header = main_body_msg.header


        if ref_frame_id is not None and ref_frame_id < len(all_pose_list):

            body_reference_pose = all_pose_list[ref_frame_id]
            for ind in self.main_body_segment_index:
                p = all_pose_list[ind]
                std_relative_pose = common.get_transform_same_base(body_reference_pose, p)
                relative_pose = common.to_ros_pose(std_relative_pose)
                main_body_msg.poses.append(relative_pose)

                all_pose_msg.poses.append(relative_pose)


                if ind == 4:  # T8
                    torso_pose_msg.pose = relative_pose
                if ind == 6:  # Head
                    head_msg.pose = relative_pose
                if ind == 10:
                    right_tcp_msg.pose = relative_pose
                if ind == 14:
                    left_tcp_msg.pose = relative_pose

            for ind in self.prop_segment_index:
                p = all_pose_list[ind]
                std_relative_pose = common.get_transform_same_base(body_reference_pose, p)
                relative_pose = common.to_ros_pose(std_relative_pose)
                prop_msg.poses.append(relative_pose)
                all_pose_msg.poses.append(relative_pose)
        else:
            for ind in self.main_body_segment_index:
                main_body_msg.poses.append(all_pose_list[ind])
                all_pose_msg.poses.append(all_pose_list[ind])
            for ind in self.prop_segment_index:
                prop_msg.poses.append(all_pose_list[ind])
                all_pose_msg.poses.append(all_pose_list[ind])

            

        if len(self.left_finger_segment_index):
            left_finger_reference_pose = all_pose_list[self.left_finger_segment_index[0]]
            for ind in self.left_finger_segment_index:
                p = all_pose_list[ind]
                std_relative_pose = common.get_transform_same_base(left_finger_reference_pose, p)
                relative_pose = common.to_ros_pose(std_relative_pose)
                left_finger_msg.poses.append(relative_pose)
                all_pose_msg.poses.append(relative_pose)


        if len(self.right_finger_segment_index):
            right_finger_reference_pose = all_pose_list[self.right_finger_segment_index[0]]
            for ind in self.right_finger_segment_index:
                p = all_pose_list[ind]
                std_relative_pose = common.get_transform_same_base(right_finger_reference_pose, p)
                relative_pose = common.to_ros_pose(std_relative_pose)
                right_finger_msg.poses.append(relative_pose)
                all_pose_msg.poses.append(relative_pose)

        pose_msg_dict = {
            'all_pose': all_pose_msg,
            'main_body': main_body_msg,
            'prop': prop_msg,
            'left_finger': left_finger_msg,
            'right_finger': right_finger_msg,
            'key_segment_torso': torso_pose_msg,
            'key_segment_left': left_tcp_msg,
            'key_segment_right': right_tcp_msg,
            'key_segment_head': head_msg
        }

        return pose_msg_dict
    

    @staticmethod
    def _decode_to_pose(item):
        """Decode a type 02 stream tself.header.iteo ROS pose message.

        :param item: str String of bytes
        """
        if len(item) != 32:
            rospy.logerr(
                "XSensInterface: Payload pose data size is not 32: {}".format(len(item))
            )
            return None
        # segment_id = common.byte_to_uint32(item[:4])
        x = common.byte_to_float(item[4:8])
        y = common.byte_to_float(item[8:12])
        z = common.byte_to_float(item[12:16])
        qw = common.byte_to_float(item[16:20])
        qx = common.byte_to_float(item[20:24])
        qy = common.byte_to_float(item[24:28])
        qz = common.byte_to_float(item[28:32])
        pose = np.array([x, y, z, qx, qy, qz, qw])
        # We do not need to convert the pose from MVN frame (x forward, y up, z right) to ROS frame,
        # since the type 02 data is Z-up, see:
        # https://www.xsens.com/hubfs/Downloads/Manuals/MVN_real-time_network_streaming_protocol_specification.pdf
        return pose



    
    @property
    def main_body_segment_index(self):
        return list(range(self.header.body_segments_num))
    


    @property
    def prop_segment_index(self):
        return list(range(self.header.body_segments_num, self.header.body_segments_num+self.header.props_num))

    @property
    def finger_segment_index(self):
        if self.header.finger_segments_num:
            return list(range(self.header.body_segments_num+self.header.props_num, self.header.body_segments_num+self.header.props_num+self.header.finger_segments_num))
        else:
            return []
    
    @property
    def left_finger_segment_index(self):
        if self.header.finger_segments_num:
            return list(range(self.header.body_segments_num+self.header.props_num, self.header.body_segments_num+self.header.props_num+20))
        else:
            return []

    @property
    def right_finger_segment_index(self):
        if self.header.finger_segments_num:
            return list(range(self.header.body_segments_num+self.header.props_num+20, self.header.body_segments_num+self.header.props_num+self.header.finger_segments_num))
        else:
            return []
    
    @property
    def item_num(self):
        return self.header.item_counter

    @property
    def item_size(self):
        """Define how many bytes in a item"""
        return self.header.payload_size // self.item_num


class XsensInterface(object):
    def __init__(
        self,
        udp_ip,
        udp_port,
        ref_frame,
        scaling=1.0,
        buffer_size=4096,
        **kwargs  # DO NOT REMOVE
    ):
        super(XsensInterface, self).__init__()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._sock.bind((udp_ip, udp_port))
        self._buffer_size = buffer_size

        self._body_frames = {
            "pelvis": 0,
            "l5": 1,
            "l3": 2,
            "t12": 3,
            "t8": 4,
            "neck": 5,
            "head": 6,
            "right_shoulder": 7,
            "right_upper_arm": 8,
            "right_forearm": 9,
            "right_hand": 10,
            "left_shoulder": 11,
            "left_upper_arm": 12,
            "left_forearm": 13,
            "left_hand": 14,
            "right_upper_leg": 15,
            "right_lower_leg": 16,
            "right_foot": 17,
            "right_toe": 18,
            "left_upper_leg": 19,
            "left_lower_leg": 20,
            "left_foot": 21,
            "left_toe": 22,
        }

        ref_frame_lowercase = ref_frame.lower()
        if ref_frame_lowercase in self._body_frames:
            self.ref_frame = ref_frame_lowercase
            self.ref_frame_id = self._body_frames[ref_frame_lowercase]
        elif ref_frame_lowercase == "" or ref_frame_lowercase == "world":
            rospy.logwarn("XSensInterface: Reference frame is the world frame")
            self.ref_frame = "world"
            self.ref_frame_id = None
        else:
            rospy.logwarn(
                "XSensInterface: Using customized reference frame {}".format(
                    ref_frame_lowercase
                )
            )
            self.ref_frame = ref_frame_lowercase
            self.ref_frame_id = None

        self.scaling_factor = scaling
        self.header = None
        self.object_poses = None
        self.all_segments_poses = None



    def get_datagram(self):
        """[Main entrance function] Get poses from the datagram."""
        data, _ = self._sock.recvfrom(self._buffer_size)
        self.datagram = self._get_datagram(data)
        if self.datagram is not None:
            pose_msg_dict = self.datagram.parsing(self.ref_frame, self.ref_frame_id)
            return pose_msg_dict
        else:
            return None


    def get_transform(self, base_frame, distal_frame):
        """Given both base frame and distal frame be two of the body frames, get the transform
        from the base frame to the distal frame as a pose.

        Args:
            base_frame: str Base frame name.
            distal_frame: str Distal frame name.

        Returns:

        """
        pose_msg = geo_msg.PoseStamped()
        base_frame = base_frame.lower()
        distal_frame = distal_frame.lower()
        pose_msg.header = self.all_segments_poses.header
        if base_frame not in self._body_frames or distal_frame not in self._body_frames:
            rospy.logerr("Base frame {} is not in known body frames".format(base_frame))
            return None
        if distal_frame not in self._body_frames:
            rospy.logerr(
                "Distal frame {} is not in known body frames".format(distal_frame)
            )
            return None
        base_frame_id = self._body_frames[base_frame]
        distal_frame_id = self._body_frames[distal_frame]
        base_pose = self.all_segments_poses.poses[base_frame_id]
        distal_pose = self.all_segments_poses.poses[distal_frame_id]
        std_pose = common.get_transform_same_base(base_pose, distal_pose)
        pose_msg = common.to_ros_pose_stamped(std_pose, base_frame)
        return pose_msg


    @staticmethod
    def _get_header(data):
        """Get the header data from the received MVN Awinda datagram.

        :param data: Tuple From self._sock.recvfrom(self._buffer_size)
        :return: Header
        """
        if len(data) < 24:
            rospy.logwarn(
                "XSensInterface: Data length {} is less than 24".format(len(data))
            )
            return None
        id_str = common.byte_to_str(data[0:6], 6)
        sample_counter = common.byte_to_uint32(data[6:10])
        datagram_counter = data[10]
        item_number = common.byte_to_uint8(data[11])
        time_code = common.byte_to_uint32(data[12:16])
        character_id = common.byte_to_uint8(data[16])
        body_segments_num = common.byte_to_uint8(data[17])
        props_num = common.byte_to_uint8(data[18])
        finger_segments_num = common.byte_to_uint8(data[19])
        # 20 21 are reserved for future use
        payload_size = common.byte_to_uint16(data[22:24])
        header = Header(
            [
                id_str,
                sample_counter,
                datagram_counter,
                item_number,
                time_code,
                character_id,
                body_segments_num,
                props_num,
                finger_segments_num,
                payload_size,
            ]
        )
        rospy.logdebug(header.__repr__())
        return header

    def _get_datagram(self, data):
        header = self._get_header(data)
        if header is not None and header.is_valid:
            self.header = header
            return Datagram(header, data[24:])
        else:
            return None
