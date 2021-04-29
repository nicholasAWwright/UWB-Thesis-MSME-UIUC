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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "decawave/pdoa_report.h"
#include "decawave/pdoa_vicon.h"

using namespace geometry_msgs;
using namespace message_filters;

std::string frame_id, filterTopic, viconTopic;

ros::Publisher pdoa_vicon_pub;


void publishCallback(const decawave::pdoa_report::ConstPtr& pdoa, const PoseStamped::ConstPtr& vicon)
{
    decawave::pdoa_vicon data_msg;

    data_msg.header.seq = 0;
    data_msg.header.stamp = vicon->header.stamp;
    data_msg.header.frame_id = vicon->header.frame_id;

    data_msg.distHorz = pdoa->distHorz;
    data_msg.pdoaHorz = pdoa->pdoaHorz;
    data_msg.distVert = pdoa->distVert;
    data_msg.pdoaVert = pdoa->pdoaVert;

    data_msg.pose.position.x = vicon->pose.position.x;
    data_msg.pose.position.y = vicon->pose.position.y;
    data_msg.pose.position.z = vicon->pose.position.z;

//  https://quaternions.online/   <-- quaternion calculator
    data_msg.pose.orientation.x = vicon->pose.orientation.x;
    data_msg.pose.orientation.y = vicon->pose.orientation.y;
    data_msg.pose.orientation.z = vicon->pose.orientation.z;
    data_msg.pose.orientation.w = vicon->pose.orientation.w;

    pdoa_vicon_pub.publish(data_msg);

    ROS_INFO("Approximate time match found");
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pdoaFiltered");
    ros::NodeHandle nh("~");

    nh.param<std::string>("frame_id", frame_id, "uwb");

    //publishes a tuple for data analysis
    pdoa_vicon_pub = nh.advertise<decawave::pdoa_vicon>("dataFiltered", 1);

    message_filters::Subscriber<decawave::pdoa_report> filter_sub(nh, "/uwb/pdoaSerial/data", 1);
    message_filters::Subscriber<PoseStamped> vicon_sub(nh, "/vicon/pdoa_tag/pose", 1);

    typedef sync_policies::ApproximateTime<decawave::pdoa_report, PoseStamped> MySyncPolicy;
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


