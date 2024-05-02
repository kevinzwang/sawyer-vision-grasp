#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped

def listener():
    rospy.init_node('point_publisher', anonymous=True)
    pubs = [
        rospy.Publisher("/cal_front_left", PointStamped, queue_size=10),
        rospy.Publisher("/cal_front_right", PointStamped, queue_size=10),
        rospy.Publisher("/cal_back_left", PointStamped, queue_size=10),
        rospy.Publisher("/cal_back_right", PointStamped, queue_size=10),
    ]

    pose_pub = rospy.Publisher("/asdfasdf", PointStamped, queue_size=10)

    points = [
        # bounding box
        [0.67637, -0.10712, -0.20349],
        [0.66894, 0.091475, -0.20366],
        [0.47837, -0.1182, -0.2013],
        [0.47034, 0.080819, -0.20084]

        #calibration
        # [-0.089167, -0.93718, 0.31221],
        # [-0.096889, -0.93623, 0.095947],
        # [-0.3683, -0.95277, 0.3175],
        # [-0.37301, -0.95263, 0.10224]
    ]

    pose_val = [-0.008, -0.0445, 0.069]

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(4):
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "base"

            point.point.x = points[i][0]
            point.point.y = points[i][1]
            point.point.z = points[i][2]

            pubs[i].publish(point)

        pose = PointStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "right_gripper_base"

        pose.point.x = pose_val[0]
        pose.point.y = pose_val[1]
        pose.point.z = pose_val[2]

        pose_pub.publish(pose)

        r.sleep()

if __name__ == '__main__':
    listener()