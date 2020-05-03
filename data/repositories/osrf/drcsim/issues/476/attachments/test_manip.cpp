#include <ros/ros.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <sstream>
#include <stdio.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_manip");
    ros::NodeHandle* rosnode = new ros::NodeHandle();

    ros::Publisher  pub_bdi1 = rosnode->advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command", 1);

    usleep(1000000);

    atlas_msgs::AtlasSimInterfaceCommand asic;
    asic.behavior = atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE;
    pub_bdi1.publish(asic);
    ros::spinOnce();
    usleep(1000000);

    return 0;
}
