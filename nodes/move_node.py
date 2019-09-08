#!/usr/bin/env python
import rospy
import numpy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import pdb

if __name__ == '__main__':
    rospy.init_node('move_turtlebot_to_goal')
    if len(sys.argv) < 6:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './move_node.py '
                     'x y z roll pitch yaw')
        sys.exit(0)
    else:
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        try:
            trans = tfBuffer.lookup_transform("odom", "base_footprint", rospy.Time(0), rospy.Duration(5.0))

            translation = trans.transform.translation
            quaternion = trans.transform.rotation
            T1 = numpy.dot(tf.transformations.quaternion_matrix((quaternion.x, quaternion.y, quaternion.z, quaternion.w)),
                           tf.transformations.translation_matrix((translation.x, translation.y, translation.z)))

            print sys.argv
            q2 = tf.transformations.quaternion_from_euler(float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]))
            T2 = numpy.dot(tf.transformations.quaternion_matrix(q2),
                           tf.transformations.translation_matrix((float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))))
            T_goal = numpy.dot(T1, T2)
            print(T1)
            print(T2)
            print(T_goal)
            pdb.set_trace()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
            print("Error", err)

        sys.exit(0)





