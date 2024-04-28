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
import numpy as np
from numpy import linalg
import grasping 
import sys
from intera_interface import gripper as robot_gripper
import utils

from intera_interface import CHECK_VERSION


def collect_points(side):
    limb = intera_interface.Limb(side)
    
    # Joint_Angles = [
    # # top
    # [-0.14012890625, -0.6778544921875, -0.08773828125, 1.5137431640625, 0.10208203125, 0.7263076171875, 1.4805322265625],
    # # front-left
    # [-0.3122373046875, 0.4198779296875, -1.3894482421875, 0.541912109375, 1.50916796875, 2.09808203125, 4.7129150390625],
    # # front
    # [-0.12808203125, 0.1749873046875, -0.9763203125, 0.2274365234375, 0.779880859375, 1.9130556640625, 4.4752763671875],
    # # iso front-right
    # [0.141486328125, 0.1496142578125, -0.9274580078125, 0.1471103515625, 0.3057783203125, 1.8706689453125, 4.0644375],
    # # right
    # [-0.026697265625, -0.3664013671875, 1.06414453125, 1.1283173828125, -1.627388671875, 2.125982421875, -2.088916015625],
    # #back-right
    # [1.218771484375, 0.2875634765625, -1.1075478515625, 1.800669921875, 0.17175, -0.0769326171875, 3.5471865234375],
    # #back
    # [0.8465771484375, 0.8729521484375, -1.073880859375, 2.083978515625, 0.6101533203125, -1.550251953125, 3.399033203125],
    # #back-left
    # [-0.22947265625, 1.1244287109375, -1.6828623046875, 1.83616796875, 1.1379267578125, -2.6620654296875, 3.455376953125],
    # #left
    # [-0.70771484375, 1.055087890625, -2.5957529296875, 1.479796875, 1.4825849609375, -2.83450390625, 3.56132421875]
    # ]
    
    Joint_Angles = [
        # top
        [0.0313544921875, -1.0435009765625, -0.407853515625, 1.9305625, 0.22340625, 0.674134765625, -1.978115234375],
        #front
        [-0.273015625, 0.360087890625, -0.5494404296875, -0.2702666015625, 0.4168251953125, 2.2443759765625, -1.6088486328125],
        #front-right
        [-0.0325634765625, 0.508552734375, -0.799623046875, -0.766392578125, 0.0492119140625, 2.404859375, -2.169248046875],
        #right
        [-0.155359375, 0.836857421875, -1.2504150390625, -1.7453251953125, -0.432494140625, 2.836201171875, -2.7187900390625],
        #back-right
        [-0.3705029296875, 0.211244140625, -2.3767724609375, -2.2592001953125, 0.6928349609375, 2.1733017578125, -1.6913115234375],
        #top
        [-0.0935048828125, -0.9056484375, -0.231021484375, 1.662990234375, 0.207642578125, 0.84210546875, 1.1422392578125],
        #back
        [-0.069513671875, -0.1634013671875, -0.728888671875, 2.283630859375, -0.63662890625, -1.737060546875, 2.181595703125],
        #back-left
        [-0.37183984375, 0.0204189453125, -0.959265625, 1.3599658203125, -1.2243349609375, -1.8574423828125, 2.284703125],
        #left
        [-0.460669921875, 0.241029296875, -1.21113671875, 0.5238564453125, -1.27721875, -2.1212255859375, 2.3774296875]
    ]
    
    while not rospy.is_shutdown():
        for joint_angle_values in Joint_Angles:
            print("Moving!!!!!!")
            joint_angle_dict = { f'right_j{i}': joint_angle_values[i] for i in range(7) }
            print(joint_angle_dict)

            r = rospy.Rate(10)
            while True:
                limb.set_joint_position_speed(0.1)
                limb.set_joint_positions(joint_angle_dict)

                true_angles = limb.joint_angles()
                if all([abs(joint_angle_dict[k] - true_angles[k]) < .01 for k in true_angles.keys()]):
                    break
                r.sleep()

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
    mesh = grasping.load_mesh(obj)
    
    grasping.custom_grasp_planner(mesh)
    grasping.visualize_grasp(mesh, best_v, curr_gripper)
    
    
    ### Find object in ar marker frame
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    
    to_frame = 'ar_marker_{}'.format(16)
    print(to_frame)
    
    # ## Find transform object frame to base frome

    if curr_gripper is None:
        print("No curr grip")
        exit()

    gripper_pose.pose.position.x = curr_gripper[0, 3]
    gripper_pose.pose.position.y = curr_gripper[1, 3]
    gripper_pose.pose.position.z = curr_gripper[2, 3]
    print(curr_gripper)
    
    # R = curr_gripper[0:3, 0:3]
    #print(R)
    quat = tf.transformations.quaternion_from_matrix(curr_gripper)
    gripper_pose.pose.orientation.x = quat[0]
    gripper_pose.pose.orientation.y = quat[1]
    gripper_pose.pose.orientation.z = quat[2]
    gripper_pose.pose.orientation.w = quat[3]

    translate_x = -0.1778
    translate_y = 0.07
    
    gripper_pose.pose.position.x +=  translate_x
    
        # Try to transform vertex to base
    try:
        trans = tfBuffer.lookup_transform('base', to_frame, rospy.Time(0), rospy.Duration(10.0))

        gripper_in_robot_base = tf2_geometry_msgs.do_transform_pose(gripper_pose, trans)
        # gripper_in_robot_base.append(vertex_in_robot_base)

    except Exception as e:
        print(e)
        print("Retrying ...")

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




def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
    See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")
    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    collect_points(args.limb)
    print("Done Collecting Points")
    grasp_object()



if __name__ == '__main__':
    main()
