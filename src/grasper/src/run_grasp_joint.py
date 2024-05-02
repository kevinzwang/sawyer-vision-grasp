#! /usr/bin/env python

import argparse
import intera_interface
import intera_external_devices
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from numpy import linalg
import grasping 
import sys
from intera_interface import gripper as robot_gripper
import utils
import open3d as o3d
from cv_bridge import CvBridge
import vedo
import cv2
from ros_numpy import numpify
import trimesh

from intera_interface import CHECK_VERSION

def grasp_object():
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def move(position, orientation):
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = position[0]
        request.ik_request.pose_stamped.pose.position.y = position[1]
        request.ik_request.pose_stamped.pose.position.z = position[2]       
        request.ik_request.pose_stamped.pose.orientation.x = orientation[0]
        request.ik_request.pose_stamped.pose.orientation.y = orientation[1]
        request.ik_request.pose_stamped.pose.orientation.z = orientation[2]
        request.ik_request.pose_stamped.pose.orientation.w = orientation[3]
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            print("plan:", plan)
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    ### GET POINTS
    gripper_pose = PoseStamped()

    ##TODO: Set mesh to point cloud mesh
    # obj = 'nozzle' # should be 'pawn' or 'nozzle'.
    # vertices, normals, poses, results = grasping.load_grasp_data(obj)
    mesh = trimesh.load_mesh('mesh.obj')
    mesh.fix_normals()
    
    print("Getting grasp!")
    best_v, curr_gripper = grasping.custom_grasp_planner_2(mesh)
    grasping.visualize_grasp(mesh, best_v, curr_gripper)
    
    
    ### Find object in ar marker frame
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # ## Find transform object frame to base frome

    if curr_gripper is None:
        print("No curr grip")
        exit()

    gripper_pose.pose.position.x = curr_gripper[0, 3]
    gripper_pose.pose.position.y = curr_gripper[1, 3]
    gripper_pose.pose.position.z = curr_gripper[2, 3]
    # print(curr_gripper)
    
    # R = curr_gripper[0:3, 0:3]
    #print(R)
    quat = tf.transformations.quaternion_from_matrix(curr_gripper)
    gripper_pose.pose.orientation.x = quat[0]
    gripper_pose.pose.orientation.y = quat[1]
    gripper_pose.pose.orientation.z = quat[2]
    gripper_pose.pose.orientation.w = quat[3]

    ### GRIPPER and MOVE

    ### CALIBRATE GRIPPER
    right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    ### TEST USING THE FIRST VERTEX
    z_offset = 0.00
    
    # ### Might need to get midpoint between vertex 1 and vertex 2
    # move_to_vertex_1 = vertices_in_robot_base[0]
    target_x = gripper_in_robot_base.pose.position.x 
    target_y = gripper_in_robot_base.pose.position.y 
    target_z = gripper_in_robot_base.pose.position.z + z_offset
    # move_to_vertex_2 = veorentationobot_base.pose.position.z 
    
    orient_grip = np.zeros((4,1))
    orient_grip[0] = gripper_pose.pose.orientation.x
    orient_grip[1] = gripper_pose.pose.orientation.y
    orient_grip[2] = gripper_pose.pose.orientation.z
    orient_grip[3] = gripper_pose.pose.orientation.w

    while not rospy.is_shutdown():
        # need some kind of offset to be above object to open gripper
        t = TransformStamped()
        t.transform.rotation.x = orient_grip[0]
        t.transform.rotation.y = orient_grip[1]
        t.transform.rotation.z = orient_grip[2]
        t.transform.rotation.w = orient_grip[3]
        
        v = Vector3Stamped()
        v.vector.z = -.10
        
        d = tf2_geometry_msgs.do_transform_vector3(v, t)
        print(target_x + d.vector.x, target_y+ d.vector.y, target_z + d.vector.z)
        # print(orient_grip)
        # move([target_x, target_y, offset_z], [orient_grip[0], orient_grip[1], orient_grip[2], orient_grip[3]])
        move([target_x + d.vector.x, target_y+ d.vector.y, target_z + d.vector.z], [gripper_pose.pose.orientation.x, gripper_pose.pose.orientation.y, gripper_pose.pose.orientation.z, gripper_pose.pose.orientation.w])
        # Open the right gripper
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')

        move([target_x, target_y, target_z], [gripper_pose.pose.orientation.x, gripper_pose.pose.orientation.y, gripper_pose.pose.orientation.z, gripper_pose.pose.orientation.w])

        # Close the right gripper
        print('Closing...')
        right_gripper.close()
        rospy.sleep(1.0)


        # move the object somewhere
        move([target_x, target_y, target_z + .3], [gripper_pose.pose.orientation.x, gripper_pose.pose.orientation.y, gripper_pose.pose.orientation.z, gripper_pose.pose.orientation.w])

        #Open the right gripper to drop the object
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')

if __name__ == '__main__':
    grasp_object()
