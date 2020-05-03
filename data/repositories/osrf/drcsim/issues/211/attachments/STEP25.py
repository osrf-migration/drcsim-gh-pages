#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('control_mode_switch')
import rospy
import time
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath

class Demo:

    def __init__(self):
        self.NUM_JOINTS = 28
        # Latest message from /atlas/atlas_sim_interface_state
        self.asis_msg = None
        # Latest message from /atlas/imu
        self.imu_msg = None

        # Set up publishers / subscribers
        self.ac_pub = rospy.Publisher('atlas/atlas_command', AtlasCommand)
        self.asic_pub = rospy.Publisher('atlas/atlas_sim_interface_command', 
          AtlasSimInterfaceCommand)
        self.asis_sub = rospy.Subscriber('atlas/atlas_sim_interface_state', 
          AtlasSimInterfaceState, self.state_cb)
        self.imu_sub = rospy.Subscriber('atlas/imu', 
          Imu, self.imu_cb)
        # Wait for subscribers to hook up, lest they miss our commands
        time.sleep(2.0)

    def state_cb(self, msg):
        self.asis_msg = msg

    def imu_cb(self, msg):
        self.imu_msg = msg

    def demo(self):
        walk_msg = AtlasSimInterfaceCommand()
        # Always insert current time
        walk_msg.header.stamp = rospy.Time.now()
        # Tell it to walk
        walk_msg.behavior = walk_msg.STEP
        walk_msg.walk_params.use_demo_walk = False
        walk_msg.step_params.use_demo_walk = False
        # Fill in some steps
        for i in range(4):
            step_data = AtlasBehaviorStepData()
            # Steps are indexed starting at 1
            step_data.step_index = i+1
            # 0 = left, 1 = right
            step_data.foot_index = i%2
            # 0.3 is a good number
            step_data.swing_height = 0.3
            # 0.63 is a good number
            step_data.duration = 0.63
            # We'll specify desired foot poses in ego-centric frame then
            # transform them into the robot's world frame.
            # Match feet so that we end with them together
            step_data.pose.position.x = (1+i/2)*0.25
            print(step_data.pose.position.x)
            # Step 0.15m to either side of center, alternating with feet
            step_data.pose.position.y = 0.15 if (i%2==0) else -0.15
            step_data.pose.position.z = 0.0
            step_data.pose.position.z = -0.1 if (i == 3) else 0.1
            print(step_data.pose.position.z)
            # Point those feet straight ahead
            step_data.pose.orientation.x = 0.0
            step_data.pose.orientation.y = 0.0
            step_data.pose.orientation.z = 0.0
            step_data.pose.orientation.w = 1.0
            # Transform this foot pose according to robot's
            # current estimated pose in the world, which is a combination of IMU
            # and internal position estimation.
            # http://www.ros.org/wiki/kdl/Tutorials/Frame%20transformations%20%28Python%29
            f1 = posemath.fromMsg(step_data.pose)
            f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.imu_msg.orientation.x,
                                                       self.imu_msg.orientation.y,
                                                       self.imu_msg.orientation.z,
                                                       self.imu_msg.orientation.w),
                             PyKDL.Vector(self.asis_msg.pos_est.position.x,
                                          self.asis_msg.pos_est.position.y,
                                          self.asis_msg.pos_est.position.z))
            f = f2 * f1
            step_data.pose = posemath.toMsg(f)
            walk_msg.walk_params.step_data[i] = step_data
            #walk_msg.walk_params.step_data[2].pose.position.z = 0.1
            if (i==0):
                walk_msg.step_params.desired_step = step_data
        print(walk_msg.step_params.desired_step.pose.position.x)
        # Use the same k_effort from the last step, to retain user control over some
        # joints. BDI has control of the other joints.
        #walk_msg.k_effort = slight_movement_msg.k_effort
        # Publish and give time to take effect
        print('[USER/BDI] Walking...')
        self.asic_pub.publish(walk_msg)
        time.sleep(8.0)

        # Step 2: Request BDI stand mode
        stand_msg = AtlasSimInterfaceCommand()
        # Always insert current time
        stand_msg.header.stamp = rospy.Time.now()
        # Tell it to stand
        #stand_msg.behavior = stand_msg.STAND_PREP
        # Set k_effort = [255] to indicate that we still want all joints under user
        # control.  The stand behavior needs a few iterations before it start
        # outputting force values.
        #stand_msg.k_effort = [255] * self.NUM_JOINTS
        # Publish and give time to take effect
        #print('[USER] Warming up BDI stand...')
        #self.asic_pub.publish(stand_msg)
        #time.sleep(1.0)
        # Now switch to stand
        stand_msg.behavior = stand_msg.STAND
        # Set k_effort = [0] to indicate that we want all joints under BDI control
        #stand_msg.k_effort = [0] * self.NUM_JOINTS
        # Publish and give time to take effect
        print('[BDI] Standing...')
        self.asic_pub.publish(stand_msg)
        time.sleep(6.0)


        # Step 5: Go back to home pose under user control
        home_msg = AtlasCommand()
        # Always insert current time
        home_msg.header.stamp = rospy.Time.now()
        # Assign some position and gain values that will get us there.
        home_msg.position = [0.0] * self.NUM_JOINTS
        home_msg.velocity = [0.0] * self.NUM_JOINTS
        home_msg.effort = [0.0] * self.NUM_JOINTS
        home_msg.kp_position = [20.0, 4000.0, 2000.0, 20.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0]
        home_msg.ki_position = [0.0] * self.NUM_JOINTS
        home_msg.kd_position = [0.0] * self.NUM_JOINTS
        # Bump up kp_velocity to reduce the jerkiness of the transition
        home_msg.kp_velocity = [50.0] * self.NUM_JOINTS
        home_msg.i_effort_min = [0.0] * self.NUM_JOINTS
        home_msg.i_effort_max = [0.0] * self.NUM_JOINTS 
        # Set k_effort = [1] to indicate that we want all joints under user control
        home_msg.k_effort = [255] * self.NUM_JOINTS
        # Publish and give time to take effect
        print('[USER] Going to home position...')
        self.ac_pub.publish(home_msg)
        time.sleep(2.0)





if __name__ == '__main__':
    rospy.init_node('control_mode_switch')
    d = Demo()
    d.demo()
