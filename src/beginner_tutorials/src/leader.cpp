#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

//leader
nav_msgs::Odometry leader_odom;

void odomCallback(const nav_msgs::Odometry &msg)
{
    leader_odom=msg;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "leader");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/mavros/local_position/pose", 1000, odomCallback);
    ros::Publisher formation_pub = n.advertise<std_msg::Int8>("target/formation", 1000);
    ros::Publisher leader_odom_pub = n.advertise<nav_msgs::Odometry>("leader/odom", 1000);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Rate loop_rate(10);
    std_msg::Int8 formation_type;
    formation_type.data=1;
    Eigen::Vector3d targetpose;
    while (ros::ok())
    {
        leader_odom_pub.publish(leader_odom);
        formation_pub.publish(formation_type);
        
        send_pos_setpoint(targetpose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}