#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped

def listener():
    rospy.init_node('point_publisher', anonymous=True)
    pubs = [
        rospy.Publisher("/cal_front_left", PointStamped, queue_size=10),
        rospy.Publisher("/cal_front_right", PointStamped, queue_size=10),
        rospy.Publisher("/cal_back_left", PointStamped, queue_size=10),
        rospy.Publisher("/cal_back_right", PointStamped, queue_size=10),
    ]

    points = [
        [0.67637, -0.10712, -0.20349],
        [0.66894, 0.091475, -0.20366],
        [0.47837, -0.1182, -0.2013],
        [0.47034, 0.080819, -0.20084]
    ]

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

        r.sleep()

if __name__ == '__main__':
    listener()