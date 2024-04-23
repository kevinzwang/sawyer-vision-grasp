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
- Start MoveIt: (run both in different terminals)
    ```
    rosrun intera_interface joint_trajectory_action_server.py
    roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
    ```