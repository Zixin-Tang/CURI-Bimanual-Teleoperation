import os
import cv2
import json
import base64
import struct
import requests
import numpy as np
from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt

try:
    import rospy
    import geometry_msgs.msg as geo_msg
    import moveit_msgs.msg as moveit_msg
    import trajectory_msgs.msg as traj_msg
except ImportError:
    print("Failed to load rospy, maybe in a virtual env without sourcing setup.bash")
    rospy = None
    geo_msg = None
    moveit_msg = None
    traj_msg = None

from utility import transform, robotics
# 定义一些颜色名称
color_names = ["red", 'green', 'blue', 'cyan', 'magenta', 'black']

flag = True

def save_pose_array_msg(msg, name):
    pose_array = np.array(msg.poses)
    np.save(os.path.join("/home/clover/Desktop", name), pose_array)


def plot_pose_array(pose_array):
    

    # 创建一个图形和3D坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ## plot all origin points
    for ind in range(pose_array.shape[0]):
        matrix = sd_pose(pose_array[ind])  # 4x4
        if ind in [1, 2, 3]:
            ax.scatter(*matrix[:3, 3], color=color_names[1])
        elif ind in [4, 5, 6, 7]:
            ax.scatter(*matrix[:3, 3], color=color_names[2])
        elif ind in [8, 9, 10, 11]:
            ax.scatter(*matrix[:3, 3], color=color_names[3])
        elif ind in [12, 13, 14, 15]:
            ax.scatter(*matrix[:3, 3], color=color_names[4])
        elif ind in [16, 17, 18, 19]:
            ax.scatter(*matrix[:3, 3], color=color_names[5])
        elif ind in [0]:
            ax.scatter(*matrix[:3, 3], color=color_names[0])
    ##

    # cnt = 0
    # for ind in [0, 1, 2, 3]:
    #     matrix = sd_pose(pose_array[ind])  # 4x4
    #     if ind:
    #         sm.SE3(matrix).plot(ax=ax, frame=str(ind), color=color_names[cnt], scale=0.02)
    #         # tmp = matrix[:3, 3] - sd_pose(pose_array[0])[:3, 3]
    #         # tmp = tmp / np.linalg.norm(tmp)
    #         # print("********************", np.dot(tmp, matrix[:3, 1]))
    #     else:
    #         sm.SE3(matrix).plot(ax=ax, frame=str(ind), color=color_names[cnt], scale=0.02)

    #     cnt = cnt + 1

    # cnt = 0
    # for ind in [1, 4, 8, 12, 16]:
    #     matrix = sd_pose(pose_array[ind])  # 4x4
    #     if ind:
    #         sm.SE3(matrix).plot(ax=ax, frame=str(ind), color=light_color_names[cnt], scale=0.02)
    #     else:
    #         sm.SE3(matrix).plot(ax=ax, frame=str(ind), color=light_color_names[cnt], scale=0.02)
    #     cnt = cnt + 1

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 关闭坐标轴
    ax.axis('off')

    # 显示图形
    plt.show()
    # plt.draw()


def to_list(values):
    """Convert a series of values in various structure to a plain python list.

    Args:
        values: PoseStamped/Pose/Quaternion/list/tuple/ndarray

    Returns:
        A list of plain python number types.
    """
    if isinstance(values, geo_msg.PoseStamped):
        output = to_list(values.pose)
    elif isinstance(values, geo_msg.Pose):
        output = [
            values.position.x,
            values.position.y,
            values.position.z,
            values.orientation.x,
            values.orientation.y,
            values.orientation.z,
            values.orientation.w,
        ]
    elif isinstance(values, geo_msg.Quaternion):
        output = [values.x, values.y, values.z, values.w]
    elif isinstance(values, list) or isinstance(values, tuple):
        output = list(values)
    elif isinstance(values, np.ndarray):
        output = values.tolist()
    else:
        raise NotImplementedError(
            "Type {} cannot be converted to list".format(type(values))
        )
    return output


def regularize_pose(pose):
    """It is odd if we do not regularize the pose

    :param pose: geometry_msgs/Pose
    :return:
    """
    pose_mat = sd_pose(pose)
    return to_ros_pose(pose_mat)


def sd_pose(pose, check=False):
    """Standardize the input pose to the 4x4 homogeneous transformation matrix SE(3).

    Args:
        pose (ndarray/list/Pose/PoseStamped): Values denoting a pose/transformation.
        check (bool): bool If true, will check if the input is legal.

    Returns:
        SE(3)

    Raises:
        ValueError if check and check failed.
        NotImplementedError if the input type is not supported.
    """
    if isinstance(pose, np.ndarray):
        if pose.ndim == 1 and pose.size == 7:
            t = pose[:3]
            q = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.quaternion_matrix(q)
            if check and not robotics.test_if_SE3(rm):
                raise ValueError
            # make sure to let tm left product rm
            return np.dot(tm, rm)
        elif pose.ndim == 1 and pose.size == 6:
            t = pose[:3]
            rpy = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.euler_matrix(rpy[0], rpy[1], rpy[2])
            if check and not robotics.test_if_SE3(rm):
                raise ValueError
            return np.dot(tm, rm)
        elif pose.shape == (4, 4):
            if check and not robotics.test_if_SE3(pose):
                raise ValueError
            return pose
        else:
            raise NotImplementedError
    elif isinstance(pose, list):
        return sd_pose(np.array(pose, dtype=float))
    elif isinstance(pose, geo_msg.Pose):
        p = pose.position
        o = pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    elif isinstance(pose, geo_msg.PoseStamped):
        p = pose.pose.position
        o = pose.pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    else:
        raise NotImplementedError


def to_ros_pose(pose, w_first=False):
    """Convert the input pose to ROS geometry msg pose

    Args:
        pose (ndarray/list/tuple): Input pose.
        w_first (bool): If true, will consider the w lies in the first place.

    Returns:
        geometry_msgs.Pose
    """
    if isinstance(pose, np.ndarray):
        msg = geo_msg.Pose()
        if pose.shape == (4, 4):
            t = transform.translation_from_matrix(pose)
            q = transform.quaternion_from_matrix(pose)
            msg.position.x = t[0]
            msg.position.y = t[1]
            msg.position.z = t[2]
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            return msg
        elif pose.size == 7:
            msg.position.x = pose[0]
            msg.position.y = pose[1]
            msg.position.z = pose[2]
            if w_first:
                msg.orientation.w = pose[3]
                msg.orientation.x = pose[4]
                msg.orientation.y = pose[5]
                msg.orientation.z = pose[6]
            else:
                msg.orientation.x = pose[3]
                msg.orientation.y = pose[4]
                msg.orientation.z = pose[5]
                msg.orientation.w = pose[6]
            return msg
        elif pose.size == 12:
            rotation_matrix = pose[:9].reshape(3, 3)
            translation = pose[9:]
            m = transform.identity_matrix()
            m[:3, :3] = rotation_matrix
            m[:3, 3] = translation
            return to_ros_pose(m)
        else:
            raise NotImplementedError
    elif isinstance(pose, list) or isinstance(pose, tuple):
        return to_ros_pose(np.asarray(pose, dtype=float), w_first)
    else:
        raise NotImplementedError


def to_ros_pose_stamped(pose, frame_id="", w_first=False):
    """Convert a pose to PoseStamped.

    Args:
        pose: list/ndarray A 1-D array of a pose.
        frame_id: str The pose's reference frame.
        w_first: bool If true, will consider the w lies in the first place.

    Returns:
        PoseStamped.
    """
    pose_stamped = geo_msg.PoseStamped()
    ros_pose = to_ros_pose(pose, w_first)
    pose_stamped.pose = ros_pose
    pose_stamped.header.frame_id = frame_id
    return pose_stamped


def to_ros_poses(poses):
    """Convert a series of poses into ROS PoseArray.

    Args:
        poses: ndarray A 2-D array containing poses.

    Returns:
        PoseArray.
    """
    msg = geo_msg.PoseArray()
    if isinstance(poses, np.ndarray):
        for pose in poses:
            ros_pose = to_ros_pose(pose)
            msg.poses.append(ros_pose)
        return msg
    else:
        raise NotImplementedError



def sd_position(position):
    if isinstance(position, np.ndarray):
        if position.shape == (3,):
            return position
        else:
            raise NotImplementedError
    elif isinstance(position, list):
        return sd_position(np.array(position))
    elif isinstance(position, geo_msg.Point):
        return sd_position(np.array([position.x, position.y, position.z]))
    else:
        raise NotImplementedError


def sd_orientation(orientation):
    """[deprecated] Standardize the input to a np array representation of orientation.

    Args:
        orientation:

    Returns:

    """
    """
    The order should be qx, qy, qz, qw.
    """
    if isinstance(orientation, np.ndarray):
        if orientation.shape == (4,):
            return orientation
        else:
            raise NotImplementedError
    elif isinstance(orientation, list):
        return sd_orientation(np.array(orientation))
    elif isinstance(orientation, geo_msg.Quaternion):
        return sd_orientation(
            np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        )
    else:
        raise NotImplementedError


def rotation_matrix_to_transformation_matrix(R):
    T = np.eye(4)
    T[:3, :3] = R
    return T


def euler_to_rotation_matrix(euler, order="XYZ", degrees=False):
    mat = R.from_euler(order, euler, degrees=degrees).as_matrix()
    return mat



def rotation_matrix_to_quat(matrix):
    quat = R.from_matrix(matrix).as_quat()
    return quat

def get_transform_same_base(start, end):
    """

    :param start: array (4, 4) Transform from the start frame to the base frame.
    :param end: array (4, 4) Transform from the end frame to the base frame.
    :return: array (4, 4) Transform from the end frame to the start frame.
    """
    sd_start = sd_pose(start)
    sd_end = sd_pose(end)
    return np.dot(np.linalg.inv(sd_start), sd_end)


def get_transform_same_target(start, end):
    """

    :param start: array (4, 4) Transform from the target frame to the start frame.
    :param end: array (4, 4) Transform from the target frame to the end frame.
    :return: array (4, 4) Transform from the end frame to the start frame.
    """
    sd_start = sd_pose(start)
    sd_end = sd_pose(end)
    return np.dot(sd_start, np.linalg.inv(sd_end))


def is_ip_valid(ip):
    """Check if an ip string is legal.

    Args:
        ip: str IP string.

    Returns:
        True if the ip is legal, False otherwise.
    """
    if not isinstance(ip, str):
        print("[Error] IP is not a string")
        return False
    if len(ip.split(".")) != 4:
        print("[Error] IP {} is illegal".format(ip))
        return False
    if ":" in ip:
        print("[Error] IP {} should not contain port number".format(ip))
        return False
    return True


def is_port_valid(port):
    """Check if the port number is legal.

    Args:
        port: int Port number.

    Returns:
        True if the number is an int, False otherwise.
    """
    if not isinstance(port, int):
        print("[Error] Port is not a int: {}".format(type(port)))
        return False
    return True




def get_param(name, value=None):
    """Get private/public param from param server.
    If the param's name does not have a leading ~, it will first be searched in private params,
    and then in public params. If with a leading ~, it will only be searched in private params.

    Args:
        name: str Param name.
        value: Any Return value if param is not set.

    Returns:
        Any Param value.
    """
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value
    




def byte_to_str(data, n):
    fmt = "!{}c".format(n)
    char_array = struct.unpack(fmt, data)
    str_out = ""
    for c in char_array:
        s = c.decode("utf-8")
        str_out += s
    return str_out




def byte_to_float(data):
    return struct.unpack("!f", data)[0]


def byte_to_uint32(data):
    return struct.unpack("!I", data)[0]

def byte_to_uint16(data):
    return struct.unpack("!H", data)[0]


def byte_to_uint8(data):
    if isinstance(data, int):
        return data
    return struct.unpack("!B", data)[0]
