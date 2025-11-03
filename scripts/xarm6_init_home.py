#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Initialize xArm6 to a consistent home position after controller spawn.
This ensures the arm starts in the same position every launch.
"""

import rospy
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Home position (radians) - default initialization positions (near zero)
HOME_POSITIONS = [0.000026, -0.000209, -0.000128, 0.000262, 0.000315, 0.000121]

def init_arm_home():
    """Send arm to home position via trajectory controller"""
    rospy.init_node('xarm6_init_home', anonymous=True)
    
    # Wait for controller to be available
    controller_topic = '/xarm6_traj_controller/command'
    rospy.loginfo("Waiting for controller topic: %s", controller_topic)
    
    pub = rospy.Publisher(controller_topic, JointTrajectory, queue_size=1)
    
    # Wait for subscriber (controller) to connect
    timeout = rospy.Duration(10.0)
    start_time = rospy.Time.now()
    while pub.get_num_connections() == 0:
        if rospy.Time.now() - start_time > timeout:
            rospy.logerr("Timeout waiting for controller subscriber")
            return False
        rospy.sleep(0.1)
    
    rospy.loginfo("Controller connected, sending home position...")
    
    # Create trajectory message
    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    point = JointTrajectoryPoint()
    point.positions = HOME_POSITIONS
    point.time_from_start = rospy.Duration(2.0)  # 2 seconds to reach home
    
    traj.points = [point]
    
    # Publish trajectory
    pub.publish(traj)
    rospy.loginfo("Sent home position trajectory: %s", HOME_POSITIONS)
    
    # Wait a bit to ensure command is received
    rospy.sleep(0.5)
    
    return True

if __name__ == '__main__':
    try:
        if init_arm_home():
            rospy.loginfo("Arm initialization complete")
        else:
            rospy.logerr("Arm initialization failed")
            sys.exit(1)
    except rospy.ROSInterruptException:
        pass

