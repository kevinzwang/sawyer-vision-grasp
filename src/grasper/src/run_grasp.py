#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_commander import MoveGroupCommander
import utils
import numpy as np

p1 = np.array([-0.089167, -0.93718, 0.31221])
p2 = np.array([-0.3683, -0.95277, 0.3175])
p3 = np.array([-0.37301, -0.95263, 0.10224])
v1 = p3-p2
v2 = p1-p2
norm = utils.normalize(utils.hat(v2)@v1)
q = np.array([norm[0], norm[1], norm[2], np.sqrt(np.linalg.norm(v1)**2 + np.linalg.norm(v2)**2) + np.dot(v1,v2)])
q = utils.normalize(q)

GRIPPER_POSES = [
    # top
    ([0.49523, 0.026, 0.069732], [1, 0, 0, 0 ])
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
            group.set_max_velocity_scaling_factor(.2)
            group.set_max_acceleration_scaling_factor(.2)

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