# Sawyer Vision Grasp

Commands you can run:

- Enable Sawyer:
    ```
    rosrun intera_interface enable_robot.py -e
    ```
- Tuck the arm: 
    ```
    roslaunch intera_examples sawyer_tuck.launch
    ```
- Publish Logitech webcam:
    ```
    roslaunch grasper sawyer_webcam.launch
    ```
- Publish Realsense webcam:
    ```
    roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 \
    color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true \
    depth_fps:=6 color_fps:=6
    ```
- Add realsense TF to Rviz
    ```
    rosrun tf static_transform_publisher 0.05 0 0.05 0 0 -0.70710678 0.70710678 /right_hand /usb_cam 1000
    ```
- Start MoveIt: (run both in different terminals)
    ```
    rosrun intera_interface joint_trajectory_action_server.py
    roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
    ```
- Run grasper
    ```
    rosrun grasper run_grasp.py
    ```
