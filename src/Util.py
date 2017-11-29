import time
import numpy as np

import tf
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf import transformations as tf_trans
from cv_bridge import CvBridge, CvBridgeError


def static_vars(**kwargs):
    """
    Neat way to declare static variables in functions
    :param kwargs: static variables
    :return:
    """

    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func

    return decorate


def pose2d_to_array(pose):
    # type: (Pose) -> list
    return [pose.position.x, pose.position.y]


def pose2d_to_array_with_angle(pose):
    # type: (Pose) -> list
    return [pose.position.x, pose.position.y, pose.orientation.z]


def pose_array_get_yaw(pose):
    # type: (list) -> float
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
    # type: (list) -> Quaternion
    quat = Quaternion()
    quat.x = arr[0]
    quat.y = arr[1]
    quat.z = arr[2]
    quat.w = arr[3]
    return quat


def quaternion_get_euler(quat):
    # type: (Quaternion) -> list
    euler_arr = np.empty(3, np.float64)
    explicit_quat = [quat.x, quat.y, quat.z, quat.w]
    euler_arr[0], euler_arr[1], euler_arr[2] = tf_trans.euler_from_quaternion(explicit_quat)
    return euler_arr


def quaternion_get_yaw(quat):
    # type: (Quaternion) -> float
    euler_arr = quaternion_get_euler(quat)
    return euler_arr[2]


def publish_tf_transformation(msg, frame, base_frame):
    # type: (Pose, basestring, basestring) -> object
    """Publishes a transformation
    """
    broadcaster = tf.TransformBroadcaster()
    if frame is None or not frame:
        return
    broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                              (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                              rospy.Time.now(),
                              frame,
                              base_frame)


@static_vars(bridge=CvBridge())
def numpy_matrix_to_rosimg(matrix, encoding):
    # type: (object, basestring) -> Image
    try:
        image = numpy_matrix_to_rosimg.bridge.cv2_to_imgmsg(matrix, encoding)
        return image
    except CvBridgeError as e:
        rospy.logerr("Error while converting numpy matrix to Image: " + str(e))

# mechanism like in matlab
def TicTocGenerator():
    ti = 0  # initial time
    tf = time.time()  # final time
    while True:
        ti = tf
        tf = time.time()
        yield tf - ti  # returns the time difference


TicToc = TicTocGenerator()  # create an instance of the TicTocGen generator


def toc(temp_bool=False):
    temp_time_interval = next(TicToc)
    if temp_bool:
        print("Elapsed time: %f seconds.\n" % temp_time_interval)
    return temp_time_interval


def tic():
    toc()


class Rate:
    def __init__(self, rate=10):
        # type: (float) -> Rate
        self.last_time = None
        self.rate = rate
        self.time = 1 / float(rate)

    def sleep(self):
        time_now = time.time()
        if self.last_time is None:
            time.sleep(self.time)
        else:
            time_diff = time_now - self.last_time
            if time_diff > self.time:
                rospy.logwarn("Won't sleep, last loop took " + str(time_diff) + " s. Loop time is " + str(self.time))
            else:
                time.sleep(time_diff)
        self.last_time = time.time()
