#!/usr/bin/python3.8
import rospy
import tf
import geometry_msgs.msg as geo_msg

from utility import common


class TFChecker:
    """
        publish transformation to TF Tree so that operator can check the retargeting poses in rviz
    """
    def __init__(self, parent_frame, child_frame):
        self.broadcaster = tf.TransformBroadcaster()
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
    
        

    def send_tf(self, pose_msg):


        # Broadcast the transformation
        self.broadcaster.sendTransform(
                                        translation=[pose_msg.position.x, pose_msg.position.y, pose_msg.position.z], 
                                        rotation=[pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w], 
                                        time=rospy.Time.now(), 
                                        child=self.child_frame, 
                                        parent=self.parent_frame
                                    )

