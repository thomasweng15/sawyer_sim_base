#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
import intera_interface
import numpy as np
from sawyer_utils.sawyer_joint_control import SawyerJointControl
import sawyer_utils.moveit_scene_utils as scene_utils
import tf.transformations as trans

base_standoff = 0.12
sensor_standoff_z = 0.1
sensor_standoff_y = 0.2

def initScene(scene, table_height, table_shape=[0.7, 1.6]):
    scene.remove_world_object("table")
    scene.remove_world_object("table1")
    # scene.remove_world_object("sensor")
    # scene.remove_world_object("sensor_rod")
    scene.remove_world_object("wall_br")

    time_stamp = rospy.get_rostime()
    
    table_center = [base_standoff + table_shape[0]/2.0, 0.0, table_height]
    table_pose = PoseStamped()
    table_pose.header.frame_id = 'base'
    table_pose.header.stamp = time_stamp
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = table_center[0]
    table_pose.pose.position.y = table_center[1]
    table_pose.pose.position.z = table_center[2]
    table_size = tuple(table_shape) + (0.0, )

    table1_center = [-0.28, -(base_standoff + table_shape[0]/2.0), table_height]
    table1_pose = PoseStamped()
    table1_pose.header.frame_id = 'base'
    table1_pose.header.stamp = time_stamp
    table1_pose.pose.orientation.w = 1.0
    table1_pose.pose.position.x = table1_center[0]
    table1_pose.pose.position.y = table1_center[1]
    table1_pose.pose.position.z = table1_center[2]
    table1_size = tuple((table_shape[1]/2, table_shape[0])) + (0.0, )


    # sensor_size = [0.15, 0.15, 0.15]
    # sensor_pose = PoseStamped()
    # sensor_pose.header.frame_id = 'camera_rgb_optical_frame'
    # sensor_pose.header.stamp = time_stamp
    # sensor_pose.pose.orientation.w = 1.0
    # sensor_pose.pose.position.x = 0
    # sensor_pose.pose.position.y = 0
    # sensor_pose.pose.position.z = 0

    # quaternion = trans.quaternion_from_euler(0, 0, -np.pi/7.5)
    # sensor_rod_size = [0.075, 1.5, 0.075]
    # sensor_rod_pose = PoseStamped()
    # sensor_rod_pose.header.frame_id = 'camera_rgb_optical_frame'
    # sensor_rod_pose.header.stamp = time_stamp
    # sensor_rod_pose.pose.orientation.x = quaternion[0]
    # sensor_rod_pose.pose.orientation.y = quaternion[1]
    # sensor_rod_pose.pose.orientation.z = quaternion[2]
    # sensor_rod_pose.pose.orientation.w = quaternion[3]
    # sensor_rod_pose.pose.position.x = 0.0
    # sensor_rod_pose.pose.position.y = 0.0
    # sensor_rod_pose.pose.position.z = -0.05

    wall_height = 1.5
    wall_b_angle = 0.0
    # wall_b_angle = 
    
    wall_b_size = (0.0, 1.05, wall_height)
    wall_br_pose = PoseStamped()
    wall_br_pose.header.frame_id = 'base'
    wall_br_pose.header.stamp = time_stamp
    wall_br_quat = trans.quaternion_about_axis(-wall_b_angle, (0,0,1))
    wall_br_pose.pose.orientation.x = wall_br_quat[0]
    wall_br_pose.pose.orientation.y = wall_br_quat[1]
    wall_br_pose.pose.orientation.z = wall_br_quat[2]
    wall_br_pose.pose.orientation.w = wall_br_quat[3]
    wall_br_pose.pose.position.x = -0.1
    wall_br_pose.pose.position.y = -0.9
    wall_br_pose.pose.position.z = wall_height/2.0 + table_center[2]
    
    rospy.sleep(0.5)
    scene.add_box('wall_br', wall_br_pose, wall_b_size)
    rospy.sleep(0.5)
    scene.add_box('table', table_pose, table_size)
    rospy.sleep(0.5)
    scene.add_box('table1', table1_pose, table1_size)
    # scene.add_box('sensor', sensor_pose, sensor_size)
    rospy.sleep(0.5)
    # scene.add_box('sensor_rod', sensor_rod_pose, sensor_rod_size)

try:
    rospy.init_node("scene_geometry")
    scene = moveit_commander.PlanningSceneInterface()
    
    sawyer = SawyerJointControl(scene)
    initScene(scene, 
                table_height=0.0,
                table_shape=[1.0, 1.6])

except rospy.ROSInterruptException:
    pass
