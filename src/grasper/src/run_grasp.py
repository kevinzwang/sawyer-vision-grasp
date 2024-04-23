#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_commander import MoveGroupCommander

GRIPPER_POSES = [
    # top
    ([0.67756, 0.029638, 0.012145], [0, 1, 0, 0]),
    # front
    ([0.73155, 0.033119, 0.027335], [0.94031, 0.036246, -0.33838, -0.0014161]),
    # iso front-right
    ([0.72166, 0.12065, 0.035301], [0.83802, 0.36586, -0.35761, 0.18969]),
    # right
    ([0.63531, 0.22698, 0.00053184], [0.62993, 0.60577, -0.34329, 0.34408]),
    # iso front-right
    ([0.72166, 0.12065, 0.035301], [0.83802, 0.36586, -0.35761, 0.18969]),
    # front
    ([0.73155, 0.033119, 0.027335], [0.94031, 0.036246, -0.33838, -0.0014161]),
    # top
    ([0.67756, 0.029638, 0.012145], [0, 1, 0, 0]),
    #top left
    ([0.63608, -0.0811, 0.047042],[0.67742, -0.65672, -0.26553, -0.19826]),
    # left
    ([0.65069, -0.14975, 0.0075623], [0.64866, -0.54784, -0.35063, -0.39519]),
     # iso back-left
    ([0.55271, -0.11101, -0.019619], [-0.43654, 0.74317, 0.21815, 0.45776]),
    # back
    ([0.50066, 0.019846, -0.00092392], [-0.06876, 0.88872, 0.02668, 0.45248])
]

    # # top safe
    # ([0.67756, 0.029638, 0.15], [0, 1, 0, 0]),

def main():
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    for pose in GRIPPER_POSES:
        input('Press [ Enter ]: ')
        
        position, orientation = pose
        
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

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0.5, 0.0])
            group.set_max_velocity_scaling_factor(.4)
            group.set_max_acceleration_scaling_factor(.4)

            # Plan IK
            plan = group.plan()
            print("plan:", plan)
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        

if __name__ == "__main__":
    main()