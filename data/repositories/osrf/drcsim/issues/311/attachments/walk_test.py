#!/usr/bin/env python
import roslib; roslib.load_manifest('trooper_footstep_controller')
import rospy
import sys

from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasBehaviorStepData
from geometry_msgs.msg import Pose, Point, Quaternion

####################################################################################
# To walk forward using multi-step, and then successfully take a single-step:
#    roslaunch atlas_utils atlas_sandia_hands.launch
#    ./walk_test straight multi
#    ./walk_test straight single
# To turn to the left using multi-step, and then unsuccessfully take a single-step:
#    roslaunch atlas_utils atlas_sandia_hands.launch
#    ./walk_test turn multi
#    ./walk_test turn single
####################################################################################

if __name__ == '__main__':
    rospy.init_node('example_client')
    pub = rospy.Publisher('/atlas/atlas_sim_interface_command',AtlasSimInterfaceCommand,latch=True)

    pose1 = Pose(Point(0.0119572058578,0.118575754778,0.0),
            Quaternion(0.000608457689539,0.00473010645478,0.129015158533,0.991631153575))
    pose2 = Pose(Point(0.381309535044,0.149473126855,0.0),
            Quaternion(0.00122065414008,0.00461022030969,0.257345067163,0.966307804107))
    pose3 = Pose(Point(0.0982685288506,0.29000297568,0.0),
            Quaternion(0.00181196712736,0.00441145134428,0.381272205596,0.924450518477))
    pose4 = Pose(Point(0.459755498394,0.356872955383,0.0),
            Quaternion(0.00237227877853,0.00413719968463,0.498676073804,0.86677530496))
    turning_steps = [pose1, pose2, pose3, pose4]

    pose5 = Pose(Point(0.27910628799,-0.12967192405,0.0),
            Quaternion(0.000198576224461,0.00422061573549,-0.00111520192293,0.999990451602))
    pose6 = Pose(Point(0.556609238139,0.173098219518,0.0),
            Quaternion(0.000198576224461,0.00422061573549,-0.00111520192293,0.999990451602))
    pose7 = Pose(Point(0.877480875244,-0.0984517208078,0.0),
            Quaternion(0.000198576224461,0.00422061573549,-0.00111520192293,0.999990451602))
    pose8 = Pose(Point(1.26350314701,0.186139407173,0.0),
            Quaternion(0.000198576224461,0.00422061573549,-0.00111520192293,0.999990451602))
    straight_steps = [pose5, pose6, pose7, pose8]
    
    goal = AtlasSimInterfaceCommand()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = '/map'
    goal.walk_params.use_demo_walk = False

    (scriptname, path_type, behavior) = rospy.myargv(argv=sys.argv)

    if behavior == 'multi':
        if path_type == "turn":
            path = turning_steps
        else:
            path = straight_steps

        steps = []
        for i in range(len(path)):
            step = AtlasBehaviorStepData()
            step.step_index = i+1
            step.foot_index = 1 if i%2==0 else 0 # alternate between left/right foot
            step.duration = 0.64 
            step.pose = path[i];
            step.swing_height = 0.2
            steps.append(step)
        goal.behavior = goal.WALK
        goal.walk_params.step_queue = steps
    else:
        goal.behavior = goal.STEP
        goal.step_params.desired_step.step_index = 1
        goal.step_params.desired_step.foot_index = 0
        goal.step_params.desired_step.duration = 0.64 
        goal.step_params.desired_step.swing_height = 0.2
        if path_type == "turn":
            goal.step_params.desired_step.pose = turning_steps[3]
        else:
            goal.step_params.desired_step.pose = straight_steps[3]

    pub.publish(goal)
    rospy.sleep(1)
