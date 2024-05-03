#!/usr/bin/env python
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

def main():

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
        # position = [0.688, 0.159, 0.383]
        # orientation = [-0.089, 0.996, 0.001, -0.016]
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

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0.5, 0.0])

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

    obj = 'nozzle' # should be 'pawn' or 'nozzle'.
    vertices, normals, poses, results = grasping.load_grasp_data(obj)
    mesh = grasping.load_mesh(obj)
    curr_gripper = None
    # for v, p in zip(vertices, poses):
    #     grasping.visualize_grasp(mesh, v, p)
    curr_gripper_list = []
    vert_list = []
    Qs = []

    # you may or may not find the code below helpful:) i was supposed to delete this from the starter code but i didn't so enjoy
    v_and_p = [grasping.custom_grasp_planner(mesh, v) for v in vertices]
    for v, p, Q in v_and_p:
        if not p is None:
            # grasping.visualize_grasp(mesh, v, p)
            curr_gripper_list.append(p)
            vert_list.append(v)
            Qs.append(Q)
        else:
            print("None")
    vert1, norm1 = grasping.randomly_sample_from_mesh(mesh, 3)
    while len(vert1) < 3:
        vert1, norm1 = grasping.randomly_sample_from_mesh(mesh, 3)
    vert2 = np.array([utils.find_intersections(mesh, vert1[i], vert1[i] - 5 * norm1[i])[0][-1] for i in range(len(vert1))]) #randomly_sample_from_mesh(mesh, 3)
    vertices = list(zip(vert1, vert2))
    
    
    curr_gripper = None
    best_v = None
    max_dot = 0
    Q_best = 0
    for i in range(len(curr_gripper_list)):
        quat = tf.transformations.quaternion_from_matrix(curr_gripper_list[i])
        if quat[1] > max_dot:
        # if Qs[i] > Q_best:
            max_dot = quat[1]
            best_v = vert_list[i]
            curr_gripper = curr_gripper_list[i]
            Q_best = Qs[i]
    
    print("Quality of Grasph {}".format(Q_best))    
    grasping.visualize_grasp(mesh, best_v, curr_gripper)
    
            
    
            
    
    ### Find object in ar marker frame
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    
    to_frame = 'ar_marker_{}'.format(16)
    print(to_frame)
    
    # ## Find transform object frame to base frome
    
    # try:
    #     trans = tfBuffer.lookup_transform('base', to_frame, rospy.Time(0), rospy.Duration(10.0))
    # except Exception as e:
    #     print(e)
    #     print("Retrying ...")

    if curr_gripper is None:
        print("No curr grip")
        exit()
    # curr_gripper = grasping.get_gripper_pose(vertices, mesh)
    # print(curr_gripper)
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




    
    # ### TRANSFORM VERTICES TO ROBOT BASE USING AR_MARKER
    # vertices_in_robot_base = []

    # for vertex in vertices:
    #     vertex_pose = PoseStamped()
    #     vertex_pose.header.frame_id = to_frame    # This is to get into ar_frame
        
    #     # Object is AR_marker but with translation
    #     translate_x = 0.07
    #     translate_y = 0.07
    #     vertex_pose.pose.position.x = vertex[0] + translate_x
    #     vertex_pose.pose.position.y = vertex[1] + translate_y
    #     vertex_pose.pose.position.z = vertex[2]

        # # Try to transform vertex to base
        # try:
        #     vertex_in_robot_base = tfBuffer.transform(vertex_pose, "base",  rospy.Duration(1.0))
        #     vertices_in_robot_base.append(vertex_in_robot_base)

        # except Exception as e:
        #     print(e)
        #     print("Retrying ...")


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
        # print('Opening...')
        # right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')
    
        
    
# Python's syntax for a main() method
if __name__ == '__main__':
    main()