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
#include "decawave/pdoa_report.h"

//#include "Eigen/Dense"
//#include "tdoa.h"
#include "trilateration.h"

#define MAX_NUM_ANCS     (4)
#define ERR (2e25)

#define PUB_RATE 100 //Hz

//Variable declarations

std::string anchor_config, tag_id, frame_id;

ros::Publisher pdoaLocate_pub; //antenna position data

ros::Subscriber localize_sub;

int count = 0;
float posXYZ[3][200] = {};
float angleTS = 0.0;
int newInput = 1;

//from MATLAB
//normalizing values
float meanX = -9.440070429252783e+03;
float stdX  =  8.213004341756646e+04;
float meanY =  4.520713553259142e+03;
float stdY  =  3.308436776992764e+04;

//horz fit values
float h00 = -0.378183283910060;
float h10 = 25.355050049927794;
float h01 = -0.647636675740480;
float h20 = -0.363285429629977;
float h11 =  0.772784612005113;
float h02 = -0.503973169839649;
float h30 =  2.614043319018768;
float h21 =  0.096359318321906;
float h12 =  1.316246547645020;
float h03 =  0.188678752943207;

//vert fit values
float v00 =  0.880016818883639;
float v10 =  0.671988602707655;
float v01 =  8.091529597002481;
float v20 = -1.523866757929391;
float v11 = -1.300085791238242;
float v02 = -0.370129459149365;
float v30 =  0.204247628643018;
float v21 =  0.832933133694471;
float v12 =  0.243333413760276;
float v03 =  0.225318359431090;

//Function prototypes

void initShield() //read shield position from vicon for position offset
{

}


void pub_PointStamped(const float X, const float Y, const float Z)
{
    geometry_msgs::PointStamped point_msg;

    point_msg.header.seq = 0;
    point_msg.header.stamp = ros::Time::now();
    point_msg.header.frame_id = frame_id;

    point_msg.point.x = X;
    point_msg.point.y = Y;
    point_msg.point.z = Z;

    pdoaLocate_pub.publish(point_msg);
}


float calcAngleHorz(long int X, long int Y)
{
    float angleHorz = 0.0;

    //normalize
    float x = (X - meanX)/stdX;
    float y = (Y - meanY)/stdY;

    //fit
    angleHorz = h00 + h10*x + h01*y + h20*pow(x,2) + h11*x*y + h02*pow(y,2) + h30*pow(x,3) + h21*pow(x,2)*y + h12*x*pow(y,2) + h03*pow(y,3);

    return angleHorz;
}

float calcAngleVert(long int X, long int Y)
{
    float angleVert = 0.0;

    //normalize
    float x = (X - meanX)/stdX;
    float y = (Y - meanY)/stdY;

    //fit
    angleVert = v00 + v10*x + v01*y + v20*pow(x,2) + v11*x*y + v02*pow(y,2) + v30*pow(x,3) + v21*pow(x,2)*y + v12*x*pow(y,2) + v03*pow(y,3);

    return angleVert;
}

// Takes in serial data from the pdoa shield and is called upon each publishing event.
// Pushes raw data through a function that outputs XYZ position
// Position data is collected and then averaged to publish a single XYZ coordinate
void localizeCallback(const decawave::pdoa_report &data_msg)
{

    long int distHorz = data_msg.distHorz;
    long int pdoaHorz = data_msg.pdoaHorz;
    long int distVert = data_msg.distVert;
    long int pdoaVert = data_msg.pdoaVert;

    //ignore the zero data separator
    if( (distHorz == 0) && (pdoaHorz == 0) && (distVert == 0) && (pdoaVert == 0) )
    {
        count = 0;
        ROS_INFO("zeros");
    }

    else if (count >= 0)
    {
        //raw data to spherical coordinates (degrees)
        float angleHorz =  calcAngleHorz(pdoaHorz,pdoaVert); //from Matlab fit
        float angleVert = -calcAngleVert(pdoaHorz,pdoaVert); //from Matlab fit
        float dist = 0.5*(distHorz + distVert);
        ROS_INFO("angleHorz[deg] = %6.1f, angleVert[deg] = %6.1f, dist[mm] = %6.0f", angleHorz, angleVert, dist);

        angleHorz += angleTS; //total station rotation from X+ in VICON frame

        //MATLAB standard spherical variables in standard units
        float r = dist/1000.0; // [m]
        float azimuth = angleHorz*M_PI/180.0; // [rad]
        float elevation = angleVert*M_PI/180.0; // [rad]


        //spherical to cartesian transformation (https://www.mathworks.com/help/matlab/ref/sph2cart.html)
        float x = r * cos(elevation) * cos(azimuth);
        float y = r * cos(elevation) * sin(azimuth);
        float z = r * sin(elevation);

        //store results in global scoped float array
        posXYZ[0][count] = x;
        posXYZ[1][count] = y;
        posXYZ[2][count] = z;

        count++; //count after array in order to access 0th element

        if(count == 200)
        {
            float avgX = 0.0;
            float avgY = 0.0;
            float avgZ = 0.0;

            for(int i = 0; i < 200; i++)
            {
                avgX += posXYZ[0][i]; //summing
                avgY += posXYZ[1][i];
                avgZ += posXYZ[2][i];
            }
            avgX /= 200.0; //averaging
            avgY /= 200.0;
            avgZ /= 200.0;

            pub_PointStamped(avgX, avgY, avgZ);

            ROS_INFO("AVERAGE: X = %6.3f, Y = %6.3f, Z = %6.3f", avgX, avgY, avgZ);
            count = -1; //flag to ignore more points
            ROS_INFO("DONE");

//            newInput = 1; //prompt to enter new TS angle
        }
    }
    else
    {
//        ROS_INFO("ESCAPE");
    }

}


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "decaLocate");
    ros::NodeHandle nh("~");

    nh.param<std::string>("anchor_config", anchor_config, "anchorPos.txt");
    nh.param<std::string>("tag_id", tag_id, "0");
    nh.param<std::string>("frame_id", frame_id, "uwb");

    pdoaLocate_pub = nh.advertise<geometry_msgs::PointStamped>("position", 1);

    localize_sub = nh.subscribe("/uwb/pdoaSerial/data", 1, localizeCallback);

    ros::Rate loop(10000);
    while(ros::ok())
    {
//        ROS_INFO("here");

        if(newInput == 1)
        {
            std::cout << "Input TS horz angle measured from VICON X+ (with a decimal point) and press Enter: ";
            std::cin >> angleTS;
            ROS_INFO("angleTS = %5.1f", angleTS);
            newInput = 0;
        }
        ros::spinOnce(); //Publilsh callbacks not called without this line
//        ros::spin();
        loop.sleep();
    }
    return 0;
}
