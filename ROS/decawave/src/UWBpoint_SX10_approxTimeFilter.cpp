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

std::string frame_id, filterTopic, trimbleTopic;

ros::Publisher uwb_pub;
ros::Publisher trimble_pub;

void pub_PointStamped(const PointStamped::ConstPtr& uwb)
{
    PointStamped point_msg;

    point_msg.header.seq = 0;
    point_msg.header.stamp = uwb->header.stamp;
    point_msg.header.frame_id = uwb->header.frame_id;

    point_msg.point.x = uwb->point.x;
    point_msg.point.y = uwb->point.y;
    point_msg.point.z = uwb->point.z;

    uwb_pub.publish(point_msg);
}

void pub_PoseStamped(const PoseStamped::ConstPtr& trimble)
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

void publishCallback(const PointStamped::ConstPtr& uwb, const PoseStamped::ConstPtr& trimble)
{
    pub_PointStamped(uwb);
    pub_PoseStamped(trimble);

    ROS_INFO("Approximate time match found");
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "filtered");
    ros::NodeHandle nh("~");

    nh.param<std::string>("frame_id", frame_id, "uwb");

    //publishes the average pose of the two tags at the average time between the points used to calculate the orientation
    trimble_pub = nh.advertise<PoseStamped>("trimble2uwb", 1);
    uwb_pub = nh.advertise<PointStamped>("uwb2trimble", 1);

    message_filters::Subscriber<PointStamped> filter_sub(nh, "/uwb/point0/decaPoint", 1);
    message_filters::Subscriber<PoseStamped> trimble_sub(nh, "/trimble/SX10/data", 1);

    typedef sync_policies::ApproximateTime<PointStamped, PoseStamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(5)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), filter_sub, trimble_sub);

    /*ros::Duration lowerBound1(0.14);
    ros::Duration lowerBound2(0.015);
    sync.setInterMessageLowerBound(0, lowerBound1);
    sync.setInterMessageLowerBound(1, lowerBound2);*/
    sync.registerCallback(boost::bind(&publishCallback, _1, _2));

    ros::spin();

    return 0;
}


