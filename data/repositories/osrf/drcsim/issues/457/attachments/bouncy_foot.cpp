#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <sstream>
#include <stdio.h>


static atlas_msgs::AtlasSimInterfaceState G_atlassiminterface_msg;


static void atlasSimInterfaceStateCallback(const atlas_msgs::AtlasSimInterfaceState::ConstPtr &msg)
{
    G_atlassiminterface_msg = *msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_fall");
    ros::NodeHandle* rosnode = new ros::NodeHandle();

    ros::SubscribeOptions simstateSo =
      ros::SubscribeOptions::create<atlas_msgs::AtlasSimInterfaceState>(
	       "/atlas/atlas_sim_interface_state", 1, atlasSimInterfaceStateCallback,
	       ros::VoidPtr(), rosnode->getCallbackQueue());
    simstateSo.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber subSimState = rosnode->subscribe(simstateSo);
    ros::Publisher  pub_bdi1 = rosnode->advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command", 1);

    usleep(1000000);

    atlas_msgs::AtlasSimInterfaceCommand asic;
    asic.behavior = atlas_msgs::AtlasSimInterfaceCommand::STEP;
    asic.walk_params.use_demo_walk = 0;
    asic.step_params.desired_step.duration = 2.5;
    asic.step_params.desired_step.foot_index = 0;
    asic.step_params.desired_step.step_index = 1;
    asic.step_params.desired_step.swing_height = 0.05;
    asic.step_params.desired_step.pose.position.x =  0.3;
    asic.step_params.desired_step.pose.position.y =  0.16;
    asic.step_params.desired_step.pose.position.z =  0.15;
    asic.step_params.desired_step.pose.orientation.x = 0.0;
    asic.step_params.desired_step.pose.orientation.y = 0.0;
    asic.step_params.desired_step.pose.orientation.z = 0.0;
    asic.step_params.desired_step.pose.orientation.w = 1.0;

    // send first step
    pub_bdi1.publish(asic);

    // Wait for transition
    while (G_atlassiminterface_msg.step_feedback.next_step_index_needed != 2) {
        ros::spinOnce();
	usleep(10000);
    }

    // send next step
    asic.step_params.desired_step.foot_index = 1;
    asic.step_params.desired_step.step_index = 2;
    asic.step_params.desired_step.pose.position.x =  0.34;
    asic.step_params.desired_step.pose.position.y = -0.06;
    asic.step_params.desired_step.pose.position.z =  0.15;
    pub_bdi1.publish(asic);
    usleep(100000);

    return 0;
}
