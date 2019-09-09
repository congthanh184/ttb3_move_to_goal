#!/usr/bin/env python
import rospy
import numpy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
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

def broadcast_goal(T_goal, frame_id, child_frame_id):
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = frame_id
    object_transform.child_frame_id = child_frame_id
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

def transform_matrix_from_params(argv):
    q = tf.transformations.quaternion_from_euler(float(argv[4]), float(argv[5]), float(argv[6]))
    T = numpy.dot(tf.transformations.translation_matrix((float(argv[1]), float(argv[2]), float(argv[3]))),
                  tf.transformations.quaternion_matrix(q))
    return T

class WorldTransform:
    def __init__(self, fixed_frame):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.fixed_frame = fixed_frame

    def poll_transform_matrix(self, target_frame):
        try:
            trans = self.tfBuffer.lookup_transform(self.fixed_frame, target_frame, rospy.Time(0), rospy.Duration(5.0))

            translation = trans.transform.translation
            quaternion = trans.transform.rotation
            T1 = numpy.dot(tf.transformations.translation_matrix((translation.x, translation.y, translation.z)),
                           tf.transformations.quaternion_matrix((quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
            return T1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
            print("Error", err)
            return []

def saturated_value(value, min_value, max_value):
    if value < min_value:
        return min_value
    elif value > max_value:
        return max_value
    return value

def ignore_small_step(value, epsilon, default_value):
    if abs(value) < epsilon:
        return default_value
    return value

if __name__ == '__main__':
    rospy.init_node('move_turtlebot_to_goal')
    if len(sys.argv) < 6:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './move_node.py '
                     'x y z roll pitch yaw')
        sys.exit(0)
    else:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        br = tf2_ros.TransformBroadcaster()
        rospy.sleep(0.5)

        world_transform_listener = WorldTransform("odom")
        world_init_pose = world_transform_listener.poll_transform_matrix("base_footprint")
        local_goal = transform_matrix_from_params(sys.argv)

        world_goal_pose = numpy.dot(world_init_pose, local_goal)

        r = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                broadcast_goal(world_goal_pose, "odom", "goal")

                world_robot_pose = world_transform_listener.poll_transform_matrix("base_footprint")
                world_origin_goal = tf.transformations.translation_from_matrix(world_goal_pose)[:-1]
                world_origin_robot = tf.transformations.translation_from_matrix(world_robot_pose)[:-1]
                world_robot_orientation = tf.transformations.euler_from_matrix(world_robot_pose)[2]

                world_goal_vector = world_origin_goal - world_origin_robot
                d_goal = numpy.sqrt((world_goal_vector**2).sum())
                d_goal = ignore_small_step(d_goal, 0.05, 0)

                theta_goal = numpy.arctan2(world_goal_vector[1], world_goal_vector[0])
                diff_angle = theta_goal - world_robot_orientation
                diff_angle = numpy.arctan2(numpy.sin(diff_angle), numpy.cos(diff_angle))
                diff_angle = ignore_small_step(diff_angle, 0.1, 0)

                k_d = 0.5
                k_theta = 0.3

                v = saturated_value(k_d * d_goal, 0, 0.07)
                if v > 0:
                    w = saturated_value(k_theta * diff_angle, -0.3, 0.3)
                else:
                    w = 0

                twist = Twist()
                twist.linear.x = v; twist.linear.y = 0.0; twist.linear.z = 0.0
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = w
                print v, w, d_goal, diff_angle
                pub.publish(twist)
                r.sleep()
        except:
            print "Error"
        finally:
            print("Finally")
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)
            r.sleep()
            pub.publish(twist)
            r.sleep()
            pub.publish(twist)
            rospy.spin()


