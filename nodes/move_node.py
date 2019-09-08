#!/usr/bin/env python
import rospy
import numpy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import pdb

def get_msg_from_matrix(object_transform, T):
    tr = tf.transformations.translation_from_matrix(T)
    object_transform.transform.translation.x = tr[0]
    object_transform.transform.translation.y = tr[1]
    object_transform.transform.translation.z = tr[2]
    q = tf.transformations.quaternion_from_matrix(T)
    object_transform.transform.rotation.x = q[0]
    object_transform.transform.rotation.y = q[1]
    object_transform.transform.rotation.z = q[2]
    object_transform.transform.rotation.w = q[3]
    return object_transform

def broadcast_goal(T_goal):
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "odom"
    object_transform.child_frame_id = "goal"
    object_transform = get_msg_from_matrix(object_transform, T_goal)
    br.sendTransform(object_transform)

def get_goal_position():
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform("odom", "base_footprint", rospy.Time(0), rospy.Duration(5.0))

        translation = trans.transform.translation
        quaternion = trans.transform.rotation
        T1 = numpy.dot(tf.transformations.translation_matrix((translation.x, translation.y, translation.z)),
                       tf.transformations.quaternion_matrix((quaternion.x, quaternion.y, quaternion.z, quaternion.w)))

        q2 = tf.transformations.quaternion_from_euler(float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]))
        T2 = numpy.dot(tf.transformations.translation_matrix((float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))), tf.transformations.quaternion_matrix(q2))
        T_goal = numpy.dot(T1, T2)

        return T_goal
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        print("Error", err)
        return []

if __name__ == '__main__':
    rospy.init_node('move_turtlebot_to_goal')
    if len(sys.argv) < 6:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './move_node.py '
                     'x y z roll pitch yaw')
        sys.exit(0)
    else:
        br = tf2_ros.TransformBroadcaster()
        rospy.sleep(0.5)

        T_goal = get_goal_position()
        while not rospy.is_shutdown():
            broadcast_goal(T_goal)
            rospy.sleep(5)

