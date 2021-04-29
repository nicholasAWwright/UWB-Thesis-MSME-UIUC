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

ros::Publisher uwb_pub;
ros::Publisher vicon_pub;

void pub_PoseWithCovarianceStamped(const PoseWithCovarianceStamped::ConstPtr& uwb)
{
    PoseWithCovarianceStamped poseCov_msg;

    poseCov_msg.header.seq = 0;
    poseCov_msg.header.stamp = uwb->header.stamp;
    poseCov_msg.header.frame_id = uwb->header.frame_id;

    poseCov_msg.pose.pose.position.x = uwb->pose.pose.position.x;
    poseCov_msg.pose.pose.position.y = uwb->pose.pose.position.y;
    poseCov_msg.pose.pose.position.z = uwb->pose.pose.position.z;

    poseCov_msg.pose.pose.orientation.x = uwb->pose.pose.orientation.x;
    poseCov_msg.pose.pose.orientation.y = uwb->pose.pose.orientation.y;
    poseCov_msg.pose.pose.orientation.z = uwb->pose.pose.orientation.z;
    poseCov_msg.pose.pose.orientation.w = uwb->pose.pose.orientation.w;

    poseCov_msg.pose.covariance = uwb->pose.covariance;

    uwb_pub.publish(poseCov_msg);
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

void publishCallback(const PoseWithCovarianceStamped::ConstPtr& uwb, const PoseStamped::ConstPtr& vicon)
{
    pub_PoseWithCovarianceStamped(uwb);
    pub_PoseStamped(vicon);

    ROS_INFO("Approximate time match found");
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "filtered");
    ros::NodeHandle nh("~");

    nh.param<std::string>("frame_id", frame_id, "uwb");

    //publishes the average pose of the two tags at the average time between the points used to calculate the orientation
    vicon_pub = nh.advertise<PoseStamped>("vicon2uwb", 1);
    uwb_pub = nh.advertise<PoseWithCovarianceStamped>("uwb2vicon", 1);

    message_filters::Subscriber<PoseWithCovarianceStamped> filter_sub(nh, "/uwb/orient/decaPose", 1);
    message_filters::Subscriber<PoseStamped> vicon_sub(nh, "/vicon/j8/pose", 1);

    typedef sync_policies::ApproximateTime<PoseWithCovarianceStamped, PoseStamped> MySyncPolicy;
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


