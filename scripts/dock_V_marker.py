from pydoc import doc
import queue
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from amr_services.srv import RotateTo, RotateToRequest
from geometry_msgs.msg import Pose, Twist


def round_to_multiple(number, multiple):
    return multiple * round(number / multiple)


def pose_cb(msg):
    global yaw_degree
    quat = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw_degree = euler[2]*180/3.14


def get_distance(point):
    return math.sqrt(point[0]**2 + point[1]**2)


def get_V_marker_pose():
    global robot_pose
    global marker_pose
    if tf_listener.frameExists("V_center") and tf_listener.frameExists("base_link"):
        try:
            tf_listener.waitForTransform(
                "map", "V_center", rospy.Time.now(), rospy.Duration(1))

            robot_pose = tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
            marker_pose = tf_listener.lookupTransform(
                "map", "V_center", rospy.Time(0))
        except:
            pass
    else:
        print("Frame V_center doesnt exist")


def get_V_side(y):
    y_offset_threshhold = 0.03
    if y > y_offset_threshhold:
        return "left"
    elif y < -y_offset_threshhold:
        return "right"
    elif y_offset_threshhold > y > -y_offset_threshhold:
        return "stright"


def call_rotate_srv(degree):
    rospy.wait_for_service('RotateTo')
    s = rospy.ServiceProxy("RotateTo", RotateTo)
    response = s(degree)


def get_direction(start, end):
    if start >= 0 and end >= 0:
        if start > end:
            return "CW"
        else:
            return "CCW"

    elif start <= 0 and end <= 0:
        if start > end:
            return "CW"
        else:
            return "CCW"

    elif start >= 0 and end <= 0:
        if start+abs(end) < 180:
            return "CW"
        else:
            return "CCW"

    elif start <= 0 and end >= 0:
        if abs(start) + end > 180:
            return "CW"
        else:
            return "CCW"


def start_dock():
    global dock_state
    global dock_angle
    x_axis_diff = marker_pose[0][0]-robot_pose[0][0]
    y_axis_diff = marker_pose[0][1]-robot_pose[0][1]
    print("X axis diff:", x_axis_diff)
    print("Y axis diff:", y_axis_diff)
    if dock_state == "START":
        print(dock_state)
        dock_state = "ADJUSTING_HEADING"
        dock_angle = round_to_multiple(yaw_degree, 90)
        print("Dock angle:", dock_angle)

    elif dock_state == "ADJUSTING_HEADING":
        print(dock_state)
        V_side = get_V_side(y_axis_diff)
        print(V_side)
        (P, I, D) = (50, 0, 0)

        tri_angle = math.atan(y_axis_diff/(0.5)) * 180 / \
            math.pi  # Stop after 0.5 in x distance
        if dock_angle == 0:
            target_angle = tri_angle
        elif dock_angle in [-180, 180]:
            target_angle = 180 - abs(tri_angle)
            if tri_angle < 0:
                target_angle = target_angle * (-1)
        else:
            print("Robot is not at dock angle")

        print("Adjusting angle:", tri_angle)
        print("Target angle:", target_angle)
        print("Angle now:", yaw_degree)
        call_rotate_srv(target_angle)
        dock_state = "ELIMINATE_Y_OFFSET"

    elif dock_state == "ELIMINATE_Y_OFFSET":
        print(dock_state)
        msg = Twist()
        if abs(y_axis_diff) > 0.01:
            msg.linear.x = 0.20
            cmd_vel.publish(msg)
        else:
            cmd_vel.publish(Twist())
            dock_state = "ON_TRACK"

    elif dock_state == "ON_TRACK":
        print(dock_state)
        call_rotate_srv(dock_angle)
        dock_state = "MOVE_FORWARD"

    elif dock_state == "MOVE_FORWARD":
        print(dock_state)
        msg = Twist()
        if abs(x_axis_diff) > 1:
            msg.linear.x = 0.2
            if abs(y_axis_diff) > 0.01:
                direction = get_direction(yaw_degree, dock_angle)
                msg.angular.z = 0.05 if direction == "CCW" else -0.05
            else:
                if abs(yaw_degree - dock_angle) > 0.5:
                    direction = get_direction(yaw_degree, dock_angle)
                    msg.angular.z = 0.05 if direction == "CCW" else -0.05
            cmd_vel.publish(msg)
        else:
            cmd_vel.publish(Twist())
            print("FINISH")
            dock_state = "START"
            rospy.set_param("/init_docking", False)
            rospy.set_param("/save_pose", True)


if __name__ == '__main__':
    rospy.init_node('dock_V_marker')
    tf_listener = tf.TransformListener()
    rospy.Subscriber("/robot_pose", Pose, pose_cb)
    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    robot_pose = None
    marker_pose = None
    dock_angle = None
    dock_state = "START"
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        get_V_marker_pose()
        if robot_pose != None and marker_pose != None and rospy.get_param("/init_docking", False):
            print("start dock")
            start_dock()
        rate.sleep()
