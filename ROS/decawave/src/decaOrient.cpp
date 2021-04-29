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

using namespace geometry_msgs;
using namespace message_filters;

std::string frame_id, decaMode;
float x_offset_0, y_offset_0, z_offset_0, x_offset_2, y_offset_2, z_offset_2;
float alpha = 0.0;

ros::Publisher decaPose_pub; //robot pose calculated from two antennas


void pub_PoseWithCovarianceStamped(const ros::Time timeStamp0,
                                   const float x, const float y, const float z,
                                   const tf2::Quaternion q)
{
    PoseWithCovarianceStamped pose_msg;

    pose_msg.header.seq = 0;
    pose_msg.header.stamp = timeStamp0;
    pose_msg.header.frame_id = frame_id;

    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    pose_msg.pose.pose.position.z = z;

//  https://quaternions.online/   <-- quaternion calculator
    pose_msg.pose.pose.orientation.x = q[0];
    pose_msg.pose.pose.orientation.y = q[1];
    pose_msg.pose.pose.orientation.z = q[2];
    pose_msg.pose.pose.orientation.w = q[3];

    pose_msg.pose.covariance = {10,  0,   0,   0,   0,   0,
                                 0, 10,   0,   0,   0,   0,
                                 0,  0, 1e6,   0,   0,   0,
                                 0,  0,   0,   1,   0,   0,
                                 0,  0,   0,   0,   1,   0,
                                 0,  0,   0,   0,   0, 1e6};

    decaPose_pub.publish(pose_msg);

    ros::Time timeStamp2 = ros::Time::now();
    ros::Duration serial2pose_dur = timeStamp2 - timeStamp0; //delay from reality to publish
    float serial2pose_sec = serial2pose_dur.toSec();
//    ROS_INFO("Time from serial data to pose publish: %fs", serial2pose_sec);
}


void orientCallback(const PointStamped::ConstPtr& point0, const PointStamped::ConstPtr& point2)
{
    //arithmetic mean of the two position readings of the center of the robot
    float x = 0.5*((point0->point.x - x_offset_0) + (point2->point.x - x_offset_2));
    float y = 0.5*((point0->point.y - y_offset_0) + (point2->point.y - y_offset_2));
    float z = 0.5*((point0->point.z - z_offset_0) + (point2->point.z - z_offset_2));

    //arithmetic mean of the two message times
    ros::Time t = point0->header.stamp + (point2->header.stamp - point0->header.stamp)*0.5;

    //angle of the tag axis measured from the uwb frame
    float total_angle = atan2(point0->point.y - point2->point.y, point0->point.x - point2->point.x);

    //robot orientation = angle of tag axis - fixed angle of tags WRT robot
    float thetaRPY = total_angle - alpha; //in radians

    tf2::Quaternion thetaQ;
    thetaQ.setRPY(0, 0, thetaRPY); //convert theta angle into a quaternion
    thetaQ.normalize(); //make sure magnitude is unity
//    ROS_INFO("%f, %f, %f",x,y,z);
//    ROS_INFO("%f", thetaQ[2]);

    pub_PoseWithCovarianceStamped(t, x, y, z, thetaQ);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "decaOrient");
    ros::NodeHandle nh("~");

    nh.param<std::string>("frame_id", frame_id, "uwb");
    nh.param<std::string>("decaMode", decaMode, "110k");

    //convention: tag0 leads tag2, where forward is along the increasingly positive x axis of the robot. 
    //Thus fore (tag0) is more positively x than aft (tag2) meausured from the robot origin

    //position of tags WRT the robot origin
    nh.param<float>("x_offset_0", x_offset_0, 0.0);
    nh.param<float>("y_offset_0", y_offset_0, 0.0);
    nh.param<float>("z_offset_0", z_offset_0, 0.0);

    nh.param<float>("x_offset_2", x_offset_2, 0.0);
    nh.param<float>("y_offset_2", y_offset_2, 0.0);
    nh.param<float>("z_offset_2", z_offset_2, 0.0);

    //publishes the average pose of the two tags at the average time between the points used to calculate the orientation
    decaPose_pub = nh.advertise<PoseWithCovarianceStamped>("decaPose", 1);

    message_filters::Subscriber<PointStamped> point0_sub(nh, "/uwb/point0/decaPoint", 1);
    message_filters::Subscriber<PointStamped> point2_sub(nh, "/uwb/point2/decaPoint", 1);

    float a = x_offset_0 - x_offset_2; //longitudinal distance parallel with x (should always be positive following stated convention)
    float b = y_offset_0 - y_offset_2; //lateral position from aft tag to fore tag
    alpha = atan2(b,a); //angle between the robot x+ axis and the axis of the uwb tags, from aft to fore

    typedef sync_policies::ApproximateTime<PointStamped, PointStamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(5)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), point0_sub, point2_sub);

    //Setting appropriate lower bound for approximateTime filter decreases time from serial to pose from 0.25s to 0.025s for 110k decaMode
    float rate_110k = 3.57; //Hz - values taken from TREK1000 user manual and seem to be best case scenario (compared with rostopic hz)
    float rate_6p8M = 10;
    ros::Duration k110(0.5*(1/rate_110k)); //half the time of the serial rate at 110k setting
    ros::Duration M6p8(0.5*(1/rate_6p8M)); //half the time of the serial rate at 110k setting

    ros::Duration lowerBound(0.0);
    if (decaMode.compare("110k") == 0)
    {
        lowerBound = k110;
    }
    else
    {
        lowerBound = M6p8;
    }

    sync.setInterMessageLowerBound(0, lowerBound);
    sync.setInterMessageLowerBound(1, lowerBound);
    sync.registerCallback(boost::bind(&orientCallback, _1, _2));

    ros::spin();

    return 0;
}


