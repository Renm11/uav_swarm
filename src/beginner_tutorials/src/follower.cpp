#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

// follower1
nav_msgs::Odometry leader_odom;
nav_msgs::Odometry my_odom;
std_msg::Int8 formation;
geometry_msgs::PoseStamped targetPose;
int i=0;

void myodomCallback(const nav_msgs::Odometry &msg)
{
    my_odom.pose.pose.position.x = msg.pose.pose.position.x;
    my_odom.pose.pose.position.y = msg.pose.pose.position.y + 2;
    my_odom.pose.pose.position.z = msg.pose.pose.position.z;
    my_odom.pose.pose.orientation.y = msg.pose.pose.orientation.x;
    my_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
    my_odom.pose.pose.orientation.y = msg.pose.pose.orientation.z;
    my_odom.pose.pose.orientation.y = msg.pose.pose.orientation.w;
}
void leaderodomCallback(const const nav_msgs::Odometry &msg)
{
    leader_odom = msg;
}
void formationCallback(const std_msg::Int8 $msg)
{
    formation = msg;
}
void computeTarget(std_msg::Int8 formation)
{
    if (formation.date == 1)
    {
        targetPose.header.seq = i;
        targetPose.header.frame_id = "world";
        targetPose.header.stamp = ros::Time();
        targetPose.pose.position.x = leader_odom.pose.pose.position.x + 4;
        targetPose.pose.position.y = my_odom.pose.pose.position.y;
        targetPose.pose.position.z = my_odom.pose.pose.position.z;
        targetPose.pose.orientation.x = my_odom.pose.pose.orientation.x;
        targetPose.pose.orientation.y = my_odom.pose.pose.orientation.y;
        targetPose.pose.orientation.z = my_odom.pose.pose.orientation.z;
        targetPose.pose.orientation.w = my_odom.pose.pose.orientation.w;
        i++;
    }
    else if (formation.data == 2)
    {

        targetPose.header.seq = i;
        targetPose.header.frame_id = "world";
        targetPose.header.stamp = ros::Time();
        targetPose.pose.position.x = leader_odom.pose.pose.position.x + 6;
        targetPose.pose.position.y = my_odom.pose.pose.position.y;
        targetPose.pose.position.z = my_odom.pose.pose.position.z;
        targetPose.pose.orientation.x = my_odom.pose.pose.orientation.x;
        targetPose.pose.orientation.y = my_odom.pose.pose.orientation.y;
        targetPose.pose.orientation.z = my_odom.pose.pose.orientation.z;
        targetPose.pose.orientation.w = my_odom.pose.pose.orientation.w;
        i++;
    }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "follower");

    ros::NodeHandle n;

    ros::Subscriber myodom_sub = n.subscribe("myodom", 1000, myodomCallback);
    ros::Subscriber myodom_sub = n.subscribe("leaderodom", 1000, leaderodomCallback);
    ros::Subscriber formation_sub = n.subscribe("target_formation", 1000, formationCallback);

    ros::Publisher targer_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        targer_pose_pub.publish(targetPose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}