import rospy
from laser_line_extraction.msg import LineSegmentList
import math
import tf
from geometry_msgs.msg import PoseStamped


def calculate_line_lenght(line):
    return math.dist(line.start, line.end)


def check_shared_point(line1, line2):
    dist1 = math.dist(line1.start, line2.end)
    dist2 = math.dist(line1.end, line2.start)
    if dist1 <= 0.05:
        return (True, line1.start)
    if dist2 <= 0.05:
        return (True, line1.end)
    return (False, None)


def check_v_shape(line1, line2, angle):
    angle_diff = math.pi - abs(line1.angle-line2.angle)
    angle_diff = angle_diff*180/math.pi
    if abs(angle_diff - angle) <= 5:
        return (True, angle_diff)
    else:
        return(False, angle_diff)


def send_tf(x, y,  child_tf, parent_tf):
    try:
        pose = PoseStamped()
        pose.header.frame_id = "front_laser_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        pose.header.stamp = rospy.Time.now() - rospy.Duration(0.1)

        global_pose = tf_listener.transformPose(parent_tf, pose)

        (x, y, z) = (global_pose.pose.position.x,
                     global_pose.pose.position.y, global_pose.pose.position.z)
        (xx, yy, zz, ww) = (global_pose.pose.orientation.x, global_pose.pose.orientation.y,
                            global_pose.pose.orientation.z, global_pose.pose.orientation.w)

        left_y = y - 0.2358
        right_y = y + 0.2358
        if abs(left_y - robot_pose[1]) < abs(right_y - robot_pose[1]):
            center_y = y - 0.2358
        else:
            center_y = y + 0.2358

        tf_broadcaster.sendTransform((x, center_y, z), (xx, yy, zz, ww),
                                     rospy.Time.now(),
                                     "station_center",
                                     parent_tf)

        tf_broadcaster.sendTransform((x, y, z), (xx, yy, zz, ww),
                                     rospy.Time.now(),
                                     child_tf,
                                     parent_tf)
    except:
        pass


def cb(msg):
    lines = msg.line_segments
    for i in range(len(lines)):
        line_lenght = calculate_line_lenght(lines[i])
        if line_lenght > 1 or line_lenght < 0.1:
            continue
        else:
            (have_shared_point, shared_point) = check_shared_point(
                lines[i], lines[i-1])
            (have_v_shape, v_angle) = check_v_shape(lines[i], lines[i-1], 100)
            if have_shared_point and have_v_shape:
                print("Found V marker with angle:", v_angle)
                send_tf(shared_point[0], shared_point[1], "V_center", "map")


def get_robot_pose_from_map():
    try:
        (trans, rot) = tf_listener.lookupTransform(
            '/map', '/base_link', rospy.Time(0))
        return trans
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return


if __name__ == '__main__':
    rospy.init_node('find_V_marker')
    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()

    robot_pose = None

    rospy.Subscriber("/line_segments", LineSegmentList, cb)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        robot_pose = get_robot_pose_from_map()
        rate.sleep()
