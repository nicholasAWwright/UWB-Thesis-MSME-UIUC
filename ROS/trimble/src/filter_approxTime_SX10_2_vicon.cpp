#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ros/ros.h"
#include "ros/package.h"
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

ros::Publisher trimble_pub;
ros::Publisher vicon_pub;

void pub_PoseStamped1(const PoseStamped::ConstPtr& trimble)
{
    PoseStamped pose_msg;

    pose_msg.header.seq = 0;
    pose_msg.header.stamp = trimble->header.stamp;
    pose_msg.header.frame_id = trimble->header.frame_id;

    pose_msg.pose.position.x = trimble->pose.position.x;
    pose_msg.pose.position.y = trimble->pose.position.y;
    pose_msg.pose.position.z = trimble->pose.position.z;

//  https://quaternions.online/   <-- quaternion calculator
    pose_msg.pose.orientation.x = trimble->pose.orientation.x;
    pose_msg.pose.orientation.y = trimble->pose.orientation.y;
    pose_msg.pose.orientation.z = trimble->pose.orientation.z;
    pose_msg.pose.orientation.w = trimble->pose.orientation.w;

    trimble_pub.publish(pose_msg);
}

void pub_PoseStamped2(const PoseStamped::ConstPtr& vicon)
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

void publishCallback(const PoseStamped::ConstPtr& trimble, const PoseStamped::ConstPtr& vicon)
{
    pub_PoseStamped1(trimble);
    pub_PoseStamped2(vicon);

    ROS_INFO("Approximate time match found");
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "filtered");
    ros::NodeHandle nh("~");

    nh.param<std::string>("frame_id", frame_id, "trimble");

    //publishes the average pose of the two tags at the average time between the points used to calculate the orientation
    trimble_pub = nh.advertise<PoseStamped>("trimble2vicon", 1);
    vicon_pub = nh.advertise<PoseStamped>("vicon2trimble", 1);

    message_filters::Subscriber<PoseStamped> filter_sub(nh, "/trimble/SX10/data", 1);
    message_filters::Subscriber<PoseStamped> vicon_sub(nh, "/vicon/j8/pose", 1);

    typedef sync_policies::ApproximateTime<PoseStamped, PoseStamped> MySyncPolicy;
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


