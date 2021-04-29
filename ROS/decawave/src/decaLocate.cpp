#include <cstdio>
#include <cstdint>
#include <cstdbool>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Header.h"
#include "ros/package.h"
#include "decawave/trek1000_report.h"

//#include "Eigen/Dense"
//#include "tdoa.h"
#include "trilateration.h"

#define MAX_NUM_ANCS     (4)
#define ERR (2e25)

#define PUB_RATE 100 //Hz

//TDOA deca_ekf;
//std::mutex ekf_mutex;

//Eigen::MatrixXf P;
//Eigen::MatrixXf A;
//Eigen::MatrixXf Q;

std::string anchor_config, tag_id, frame_id;

ros::Publisher decaPoint_pub; //antenna position data
ros::Publisher decaPointAsPose_pub; //antenna position with arbitrary pose for robot_localization input
//ros::Publisher decaPose_pub; //robot pose calculated from two antennas //TODO: move to orient_engine

ros::Subscriber localize_sub;
//ros::Subscriber orient_sub; //TODO: move this to orientation engine

//Function prototypes
//void initRobotMatrices(std::string type);

vec3d anchorArray[4]; //anchor locations

void initAnchors()//(TDOA &ekf) //reads anchor positions from .txt file in config folder
{
    std::string path = ros::package::getPath("decawave");
    std::ifstream file(path+"/config/"+ anchor_config);
    std::string str;
    float x, y, z;
    int i = 0;
    while (std::getline(file, str))
    {
        sscanf(str.c_str(), "%f, %f, %f", &x,&y,&z);
//        ekf.setAncPosition(i, x, y, z); //c++ extended kalman filter

        anchorArray[i].x = x; //physical anchor locations
        anchorArray[i].y = y;
        anchorArray[i].z = z;

        i++;
    }
}


void pub_PointStamped(const vec3d p, const ros::Time timeStamp0)
{
    geometry_msgs::PointStamped point_msg;

    point_msg.header.seq = 0;
    point_msg.header.stamp = timeStamp0;
    point_msg.header.frame_id = frame_id;

    point_msg.point.x = p.x;
    point_msg.point.y = p.y;
    point_msg.point.z = p.z;

    decaPoint_pub.publish(point_msg);
}


//void pub_PoseWithCovarianceStamped(const vec3d_t p, const rowMajor_t c) //NickW modified to publish a stamped point msg
void pub_PoseWithCovarianceStamped(const vec3d p, const ros::Time timeStamp0) //note vec3d vs vec3d_t
{
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.seq = 0;
    pose_msg.header.stamp = timeStamp0;
    pose_msg.header.frame_id = frame_id;

    pose_msg.pose.pose.position.x = p.x;
    pose_msg.pose.pose.position.y = p.y;
    pose_msg.pose.pose.position.z = p.z;
//  https://quaternions.online/   <-- quaternion calculator
    pose_msg.pose.pose.orientation.x = 0; // this quaternion points along positive z-axis
    pose_msg.pose.pose.orientation.y = -0.707107;
    pose_msg.pose.pose.orientation.z = 0;
    pose_msg.pose.pose.orientation.w = 0.707107;

    pose_msg.pose.covariance = {10,  0,   0,   0,   0,   0,
                                 0, 10,   0,   0,   0,   0,
                                 0,  0, 1e6,   0,   0,   0,
                                 0,  0,   0, 1e6,   0,   0,
                                 0,  0,   0,   0, 1e6,   0,
                                 0,  0,   0,   0,   0, 1e6};

    decaPointAsPose_pub.publish(pose_msg);
}


vec3d trilaterateTag(const int validRanges, const int *rangeArray)
{
    vec3d report;
    report.x = 0;
    report.y = 0;
    report.z = 0;

    int range[4];
    range[0] = rangeArray[0];
    range[1] = rangeArray[1];
    range[2] = rangeArray[2];
    range[3] = rangeArray[3];

    int nolocation = 0;
    int result = 0;

// std::cout << report.y << std::endl;

    if(validRanges >= 3) //check for enough valid distances to trilaterate
    {
        result = GetLocation(&report, ((validRanges==4) ? 1 : 0), &anchorArray[0], &range[0]); //calls trilateration.cpp
        if(result >= 0)
        {
            return report;
        }
        else //no solution
        {
            return {ERR,ERR,ERR};
        }
    }
}



void localizeCallback(const decawave::trek1000_report &data_msg)
{
    int range[4];
    range[0] = data_msg.range[0];
    range[1] = data_msg.range[1];
    range[2] = data_msg.range[2];
    range[3] = data_msg.range[3];

    if(data_msg.mID == 99) //these reports relate to corrected tag <-> anchor ranges (ascii 99 = c)
    {
        uint8_t count = 0;
        //check the mask and process the tag - anchor ranges
        for(uint8_t k=0; k<MAX_NUM_ANCS; k++) // k goes up to 3 here
        {
            if(data_msg.range[k] != 0) //we have a valid range
            {
                count++;
            }
        }
        if (data_msg.mask == 7 || data_msg.mask == 15) //anchors are 0000 if none on, 1111 if all on, need 1110 or 1111
        {
//            std::cout << "range(mm): " << "A0:" << range[0] << " A1:" << range[1] << " A2:" << range[2] << " A3:" << range[3] << std::endl;
//            ROS_INFO("%d",count);
            vec3d TWRpos = trilaterateTag(count, &range[0]); //returns tag position in meters


//                     * ************** *
//                     * Publish to ROS *
//                     * ************** *
            if (TWRpos.z != ERR) //publish only valid readings
            {
                //std::cout << mask << std::endl;
                pub_PoseWithCovarianceStamped(TWRpos, data_msg.header.stamp); //NickW TWR
                pub_PointStamped(TWRpos, data_msg.header.stamp); //NickW from pub_state originally to pub_PointStamped
                //std::cout << "X: " << TWRpos.x << ", Y: " << TWRpos.y << std::endl;
            }
            else
            {
                ROS_INFO("trilateration failed: tag%s", tag_id.c_str());
//                std::cout << "trilateration failed" << std::endl;
            }
        }
        else
        {
            ROS_INFO("invalid anchor mask: tag%s", tag_id.c_str());
//            std::cout << "invalid anchor mask" << std::endl;
        }

    }
}


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "decaLocate");
    ros::NodeHandle nh("~");

    nh.param<std::string>("anchor_config", anchor_config, "anchorPos.txt");
    nh.param<std::string>("tag_id", tag_id, "0");
    nh.param<std::string>("frame_id", frame_id, "uwb");
//    nh.param<std::string>("trilateration_method", triLat_algo, "trek1000_default");
//    nh.param<std::string>("orientation", orient, "off");

    decaPoint_pub = nh.advertise<geometry_msgs::PointStamped>("decaPoint", 1);
    decaPointAsPose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("decaPointAsPose", 1);
    localize_sub = nh.subscribe("/uwb/serial" + tag_id + "/decaData", 3, localizeCallback); //buffer is size 3 for the potential message IDs that trek1000 may return quickly (seems to be taking up too much bandwidth within this file)
    
//    //c++ extended kalman filter
//    initRobotMatrices(robot_type);
//    vec3d_t initial_position = {0,0,0};
//    //TDOA deca_ekf(A, P, Q, initial_position);
//    deca_ekf.setPredictionMat(P);
//    deca_ekf.setTransitionMat(A);
//    deca_ekf.setCovarianceMat(Q);
    
//    initAnchors(deca_ekf); //original
    initAnchors();

//    std::cout << anchorArray[0].x << std::endl;
    
//    ros::Rate loop(100);

    while(ros::ok())
    {
//ROS_INFO("here");

//        //c++ extended kalman filter
//        ekf_mutex.lock();
//        deca_ekf.stateEstimatorPredict(1./PUB_RATE);
//        deca_ekf.stateEstimatorAddProcessNoise();
//        deca_ekf.stateEstimatorFinalize();
		
//        vec3d_t pos = deca_ekf.getLocation();
//        vec3d_t vel = deca_ekf.getVelocity();
//        rowMajor_t cov = deca_ekf.getCovariance();
//        ekf_mutex.unlock();
        ros::spinOnce(); //Publilsh callbacks not called without this line
        ros::spin();
//        loop.sleep();
    }
    return 0;
}

//c++ extended kalman filter
//void initRobotMatrices(std::string type)
//{
//    if (type == "car")
//    {
//        A.setIdentity(6,6);
//        A(5,5) = 0; //No z velocity
        
//        P.setZero(6,6);
//        P(0,0) = 10000;
//        P(1,1) = 10000;
//        P(2,2) = 0;
//        P(3,3) = 1e-4;
//        P(4,4) = 1e-4;
//        P(5,5) = 0;
        
//        Q.setZero(6,6);
//    }
//    else if (type == "quadcopter")
//    {
//        A.setIdentity(6,6);

//        P.setZero(6,6);
//        P(0,0) = 10000;
//        P(1,1) = 10000;
//        P(2,2) = 100;
//        P(3,3) = 1e-4;
//        P(4,4) = 1e-4;
//        P(5,5) = 1e-4;
        
//        Q.setZero(6,6);
//    }
//    else
//    {
//        A.setIdentity(6,6);
        
//        P.setZero(6,6);
//        P(0,0) = 10000;
//        P(1,1) = 10000;
//        P(2,2) = 100;
//        P(3,3) = 1e-4;
//        P(4,4) = 1e-4;
//        P(5,5) = 1e-4;
        
//        Q.setZero(6,6);
//    }
//}
