#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def map_keyboard(side):
    limb = intera_interface.Limb(side)

    done = False
    
    while not done and not rospy.is_shutdown():
        joint_angle_input = input("joint angles (new line to exit): ")
        if joint_angle_input == '':
            done = True
            rospy.signal_shutdown("Example finished.")
        else:
            joint_angle_values = [float(x) for x in joint_angle_input.split()]
            if len(joint_angle_values) != 7:
                print("????")
                continue
            joint_angle_dict = { f'right_j{i}': joint_angle_values[i] for i in range(7) }

            r = rospy.Rate(10)
            while True:
                limb.set_joint_position_speed(0.1)
                limb.set_joint_positions(joint_angle_dict)

                true_angles = limb.joint_angles()
                if all([abs(joint_angle_dict[k] - true_angles[k]) < .01 for k in true_angles.keys()]):
                    break
                r.sleep()


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
    map_keyboard(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
