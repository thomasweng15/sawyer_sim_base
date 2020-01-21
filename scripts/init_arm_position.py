#!/usr/bin/env python

import rospy
import intera_interface

rospy.init_node("init_arm_sim")
limb = intera_interface.Limb('right')
starting_joint_angles = {'right_j0': -0.041662954890248294,
                        'right_j1': -1.0258291091425074,
                        'right_j2': 0.0293680414401436,
                        'right_j3': 2.17518162913313,
                        'right_j4':  -0.06703022873354225,
                        'right_j5': 0.3968371433926965,
                        'right_j6': 1.7659649178699421}
limb.move_to_joint_positions(starting_joint_angles)
