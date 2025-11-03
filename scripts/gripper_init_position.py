#!/usr/bin/env python

from __future__ import print_function

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time

def init_gripper_position():
    """Send initial position command to gripper to hold current position (normalized)"""
    rospy.init_node('gripper_init_position', anonymous=True)
    
    import math
    
    # Wait for joint_states to be available
    rospy.loginfo("Waiting for /joint_states to get current gripper position...")
    try:
        joint_state_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /joint_states topic")
        return
    
    # Find drive_joint position
    try:
        drive_joint_idx = joint_state_msg.name.index('drive_joint')
        raw_position = joint_state_msg.position[drive_joint_idx]
        rospy.loginfo("Raw gripper position from joint_states: {:.3f} rad".format(raw_position))
        
        # Check if position is in reasonable range [0, 1.5] rad
        # If outside, use safe default 0.0 (gripper open)
        if 0.0 <= raw_position <= 1.5:
            target_position = raw_position
            rospy.loginfo("Using current gripper position: {:.3f} rad (within safe range)".format(raw_position))
        else:
            # Position is way out of range (e.g., 30 rad) - use safe default
            target_position = 0.0
            rospy.logwarn("Current position {:.3f} rad is OUTSIDE safe range [0, 1.5], sending safe default 0.0 (open)".format(raw_position))
        
    except ValueError:
        rospy.logwarn("drive_joint not found in joint_states, using default 0.0")
        target_position = 0.0
    
    current_position = target_position
    
    # Wait for gripper controller to be ready
    rospy.loginfo("Waiting for gripper controller to be ready...")
    pub = rospy.Publisher('/gripper_traj_controller/command', JointTrajectory, queue_size=1)
    
    # Wait for subscriber
    rate = rospy.Rate(10)
    timeout = rospy.Time.now() + rospy.Duration(5.0)
    while pub.get_num_connections() == 0 and rospy.Time.now() < timeout:
        rate.sleep()
    
    if pub.get_num_connections() == 0:
        rospy.logerr("Gripper controller not ready!")
        return
    
    # Create trajectory message to hold current position
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ['drive_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [current_position]
    point.velocities = [0.0]
    point.time_from_start = rospy.Duration(0.1)
    
    trajectory_msg.points = [point]
    trajectory_msg.header.stamp = rospy.Time.now()
    
    # Send command multiple times to ensure it's received
    rospy.loginfo("Sending initial gripper position command: {:.3f} rad".format(current_position))
    for i in range(10):
        trajectory_msg.header.stamp = rospy.Time.now()
        pub.publish(trajectory_msg)
        rospy.sleep(0.1)
    
    rospy.loginfo("Gripper initial position sent successfully")
    rospy.sleep(1.0)  # Wait a bit before exiting

if __name__ == '__main__':
    try:
        init_gripper_position()
    except rospy.ROSInterruptException:
        pass

