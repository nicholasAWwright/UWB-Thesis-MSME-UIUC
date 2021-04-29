//#include <cstdio>
//#include <cstdint>
//#include <cstdbool>
//#include <cstdlib>
//#include <cstring>
//#include <cmath>
//#include <csignal>
//#include <iostream>
//#include <fstream>
//#include <chrono>
//#include <thread>
//#include <mutex>
//#include <string>

//#include "ros/ros.h"
//#include "ros/package.h"


#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_msgs/Header.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

using namespace geometry_msgs;
using namespace message_filters;

std::string frame_id, filterTopic, viconTopic;

ros::Publisher pdoa_pub;
ros::Publisher vicon_pub;

void pub_PointStamped(const PointStamped::ConstPtr& pdoa)
{
    PointStamped point_msg;

    point_msg.header.seq = 0;
    point_msg.header.stamp = pdoa->header.stamp;
    point_msg.header.frame_id = pdoa->header.frame_id;

    point_msg.point.x = pdoa->point.x;
    point_msg.point.y = pdoa->point.y;
    point_msg.point.z = pdoa->point.z;

    pdoa_pub.publish(point_msg);
}

void pub_PoseStamped(const PoseStamped::ConstPtr& vicon)
{
    PoseStamped pose_msg;

    pose_msg.header.seq = 0;
    pose_msg.header.stamp = vicon->header.stamp;
    pose_msg.header.frame_id = vicon->header.frame_id;

    pose_msg.pose.position.x = vicon->pose.position.x;
    pose_msg.pose.position.y = vicon->pose.position.y;
    pose_msg.pose.position.z = vicon->pose.position.z;

//  https://quaternions.online/   <-- quaternion calculator
    pose_msg.pose.orientation.x = vicon->pose.orientation.x;
    pose_msg.pose.orientation.y = vicon->pose.orientation.y;
    pose_msg.pose.orientation.z = vicon->pose.orientation.z;
    pose_msg.pose.orientation.w = vicon->pose.orientation.w;

    vicon_pub.publish(pose_msg);
}

void publishCallback(const PointStamped::ConstPtr& pdoa, const PoseStamped::ConstPtr& vicon)
{
    pub_PointStamped(pdoa);
    pub_PoseStamped(vicon);

    ROS_INFO("Approximate time match found");
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pdoaFiltered");
    ros::NodeHandle nh("~");

    nh.param<std::string>("frame_id", frame_id, "uwb");

    //publishes the average pose of the two tags at the average time between the points used to calculate the orientation
    vicon_pub = nh.advertise<PoseStamped>("vicon2pdoaLocator", 1);
    pdoa_pub = nh.advertise<PointStamped>("pdoaLocator2vicon", 1);

    message_filters::Subscriber<PointStamped> filter_sub(nh, "/uwb/pdoaLocator/position", 1);
    message_filters::Subscriber<PoseStamped> vicon_sub(nh, "/vicon/pdoa_tag/pose", 1);

    typedef sync_policies::ApproximateTime<PointStamped, PoseStamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(5)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(120), filter_sub, vicon_sub);

    /*ros::Duration lowerBound1(0.14);
    ros::Duration lowerBound2(0.015);
    sync.setInterMessageLowerBound(0, lowerBound1);
    sync.setInterMessageLowerBound(1, lowerBound2);*/
    sync.registerCallback(boost::bind(&publishCallback, _1, _2));

    ros::spin();

    return 0;
}


