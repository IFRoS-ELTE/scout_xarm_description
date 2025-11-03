#!/usr/bin/env python

from __future__ import print_function

import threading
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
from select import select
import termios
import tty
import copy

# Joint names for xArm6
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

msg = """
XArm6 Keyboard Teleoperation
----------------------------
Control individual joints:
  Joint 1 (Base):        q/a  (yaw left/right)
  Joint 2 (Shoulder):    w/s  (up/down)
  Joint 3 (Elbow):       e/d  (up/down)
  Joint 4 (Wrist roll):  r/f  (roll)
  Joint 5 (Wrist pitch): t/g  (pitch)
  Joint 6 (Wrist yaw):   y/h  (yaw)

Movement speed:
  u/j : increase/decrease all joint speeds by 10%
  i/k : increase/decrease current joint speed by 10%

Special:
  z   : reset all joints to 0 position
  x   : home position (typical safe position)
  c   : current speed status
  
CTRL-C to quit
"""

# Default joint positions (radians) - typical home position
HOME_POSITIONS = [0.0, -0.5236, 1.0472, 0.0, 0.5236, 0.0]  # ~30deg, 60deg, 30deg offsets

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        # Controller is in root namespace (scout's gazebo_ros_control uses root namespace)
        self.publisher = rospy.Publisher('/xarm6_traj_controller/command', JointTrajectory, queue_size=1)
        self.joint_positions = [0.0] * 6
        self.joint_speeds = [0.5] * 6  # rad/s per joint
        self.default_speed = 0.5
        self.condition = threading.Condition()
        self.done = False
        
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
            
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update_joint(self, joint_idx, delta):
        self.condition.acquire()
        self.joint_positions[joint_idx] += delta * self.joint_speeds[joint_idx]
        self.condition.notify()
        self.condition.release()

    def set_joint(self, joint_idx, value):
        self.condition.acquire()
        self.joint_positions[joint_idx] = value
        self.condition.notify()
        self.condition.release()

    def reset_all_joints(self):
        self.condition.acquire()
        self.joint_positions = [0.0] * 6
        self.condition.notify()
        self.condition.release()

    def home_position(self):
        self.condition.acquire()
        self.joint_positions = copy.deepcopy(HOME_POSITIONS)
        self.condition.notify()
        self.condition.release()

    def update_speed(self, joint_idx, factor):
        self.condition.acquire()
        self.joint_speeds[joint_idx] = max(0.01, min(2.0, self.joint_speeds[joint_idx] * factor))
        self.condition.release()

    def update_all_speeds(self, factor):
        self.condition.acquire()
        for i in range(6):
            self.joint_speeds[i] = max(0.01, min(2.0, self.joint_speeds[i] * factor))
        self.condition.release()

    def get_speeds(self):
        self.condition.acquire()
        speeds = copy.deepcopy(self.joint_speeds)
        self.condition.release()
        return speeds

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES
        
        while not self.done:
            self.condition.acquire()
            # Wait for update or timeout
            self.condition.wait(self.timeout)

            # Create trajectory point with current positions
            point = JointTrajectoryPoint()
            point.positions = copy.deepcopy(self.joint_positions)
            point.velocities = [0.0] * 6
            point.time_from_start = rospy.Duration(0.1)  # Short duration for responsive control

            trajectory_msg.points = [point]
            trajectory_msg.header.stamp = rospy.Time.now()

            self.condition.release()

            # Publish trajectory
            self.publisher.publish(trajectory_msg)

def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def print_speeds(speeds):
    print("Current joint speeds (rad/s):")
    for i, speed in enumerate(speeds):
        print("  Joint {}: {:.3f}".format(i+1, speed))

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('xarm6_keyboard_teleop')

    default_speed = rospy.get_param("~default_speed", 0.5)
    repeat_rate = rospy.get_param("~repeat_rate", 10.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = PublishThread(repeat_rate)
    
    # Initialize speeds
    pub_thread.joint_speeds = [default_speed] * 6
    pub_thread.default_speed = default_speed
    
    current_joint = 0  # Track which joint is being controlled

    try:
        pub_thread.wait_for_subscribers()
        print(msg)
        print_speeds(pub_thread.get_speeds())
        
        while(1):
            key = getKey(settings, key_timeout)
            
            if key == '':
                continue
                
            # Joint 1 controls (q/a)
            if key == 'q':
                pub_thread.update_joint(0, 1.0)  # Increase joint1
            elif key == 'a':
                pub_thread.update_joint(0, -1.0)  # Decrease joint1
                
            # Joint 2 controls (w/s)
            elif key == 'w':
                pub_thread.update_joint(1, 1.0)  # Increase joint2
            elif key == 's':
                pub_thread.update_joint(1, -1.0)  # Decrease joint2
                
            # Joint 3 controls (e/d)
            elif key == 'e':
                pub_thread.update_joint(2, 1.0)  # Increase joint3
            elif key == 'd':
                pub_thread.update_joint(2, -1.0)  # Decrease joint3
                
            # Joint 4 controls (r/f)
            elif key == 'r':
                pub_thread.update_joint(3, 1.0)  # Increase joint4
            elif key == 'f':
                pub_thread.update_joint(3, -1.0)  # Decrease joint4
                
            # Joint 5 controls (t/g)
            elif key == 't':
                pub_thread.update_joint(4, 1.0)  # Increase joint5
            elif key == 'g':
                pub_thread.update_joint(4, -1.0)  # Decrease joint5
                
            # Joint 6 controls (y/h)
            elif key == 'y':
                pub_thread.update_joint(5, 1.0)  # Increase joint6
            elif key == 'h':
                pub_thread.update_joint(5, -1.0)  # Decrease joint6
                
            # Speed controls
            elif key == 'u':
                pub_thread.update_all_speeds(1.1)
                print_speeds(pub_thread.get_speeds())
            elif key == 'j':
                pub_thread.update_all_speeds(0.9)
                print_speeds(pub_thread.get_speeds())
            elif key == 'i':
                pub_thread.update_speed(current_joint, 1.1)
                print("Joint {} speed: {:.3f} rad/s".format(current_joint + 1, pub_thread.joint_speeds[current_joint]))
            elif key == 'k':
                pub_thread.update_speed(current_joint, 0.9)
                print("Joint {} speed: {:.3f} rad/s".format(current_joint + 1, pub_thread.joint_speeds[current_joint]))
                
            # Special commands
            elif key == 'z':
                pub_thread.reset_all_joints()
                print("Reset all joints to 0")
            elif key == 'x':
                pub_thread.home_position()
                print("Move to home position")
            elif key == 'c':
                print_speeds(pub_thread.get_speeds())
                
            elif key == '\x03':  # CTRL-C
                break

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
        print("\nTeleop stopped.")

