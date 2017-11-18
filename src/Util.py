import numpy as np

import tf
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf import transformations as tf_trans



def pose2d_to_array(pose):
    return [pose.position.x, pose.position.y]


def pose2d_to_array_with_angle(pose):
    return [pose.position.x, pose.position.y, pose.orientation.z]


def pose_array_get_yaw(pose):
    # type: ([]) -> float
    len_pose = len(pose)
    if len_pose == 3:
        return pose[2]
    elif len_pose == 6:
        return pose[5]
    elif len_pose == 7:
        quat_array = pose[3:7]
        quat = array_to_quaternion(quat_array)
        return quaternion_get_yaw(quat)


def array_to_quaternion(arr):
    # type: ([]) -> Quaternion
    quat = Quaternion()
    quat.x = arr[0]
    quat.y = arr[1]
    quat.z = arr[2]
    quat.w = arr[3]
    return quat


def quaternion_get_euler(quat):
    # type: (Quaternion) -> []
    euler_arr = np.empty(3, np.float64)
    explicit_quat = [quat.x, quat.y, quat.z, quat.w]
    euler_arr[0], euler_arr[1], euler_arr[2] = tf_trans.euler_from_quaternion(explicit_quat)
    return euler_arr


def quaternion_get_yaw(quat):
    # type: (Quaternion) -> float
    euler_arr = quaternion_get_euler(quat)
    return euler_arr[2]


def publish_tf_transformation(msg, frame, base_frame):
    """Publishes a transformation"""
    broadcaster = tf.TransformBroadcaster()
    if frame is None or not frame:
        return
    broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                              (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                              rospy.Time.now(),
                              frame,
                              base_frame)

