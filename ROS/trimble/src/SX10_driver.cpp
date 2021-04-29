#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "serial/serial.h"

#define SPEED         115200 //serial baud rate

#define TOF_REPORT_LEN  (65) // \r and \n add 2 to length of visible 63
#define TOF_REPORT_ARGS (12)

std::string port, tag_id, frame_id; //ROS params
ros::Publisher trimbleData_pub; //cartesian data
ros::Publisher trimbleDataRaw_pub; //raw spherical data


void pub_poseStamped(const float x, const float y, const float z, const ros::Time timeStamp0)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.seq = 0;
    pose_msg.header.stamp = timeStamp0;
    pose_msg.header.frame_id = frame_id;

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;
//  https://quaternions.online/   <-- quaternion calculator
    pose_msg.pose.orientation.x = 0.0; // this quaternion points along positive x-axis
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;

//    pose_msg.pose.covariance = {10,  0,   0,   0,   0,   0,
//                                 0, 10,   0,   0,   0,   0,
//                                 0,  0, 1e6,   0,   0,   0,
//                                 0,  0,   0, 1e6,   0,   0,
//                                 0,  0,   0,   0, 1e6,   0,
//                                 0,  0,   0,   0,   0, 1e6};

    trimbleData_pub.publish(pose_msg);
}

void pub_pointStamped(const float x, const float y, const float z, const ros::Time timeStamp0)
{
    geometry_msgs::PointStamped point_msg;

    point_msg.header.seq = 0;
    point_msg.header.stamp = timeStamp0;
    point_msg.header.frame_id = frame_id;

    point_msg.point.x = x;
    point_msg.point.y = y;
    point_msg.point.z = z;
////  https://quaternions.online/   <-- quaternion calculator
//    pose_msg.pose.orientation.x = 0.0; // this quaternion points along positive x-axis
//    pose_msg.pose.orientation.y = 0.0;
//    pose_msg.pose.orientation.z = 0.0;
//    pose_msg.pose.orientation.w = 1.0;

//    pose_msg.pose.covariance = {10,  0,   0,   0,   0,   0,
//                                 0, 10,   0,   0,   0,   0,
//                                 0,  0, 1e6,   0,   0,   0,
//                                 0,  0,   0, 1e6,   0,   0,
//                                 0,  0,   0,   0, 1e6,   0,
//                                 0,  0,   0,   0,   0, 1e6};

    trimbleDataRaw_pub.publish(point_msg);
}


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "trimble");
    ros::NodeHandle nh("~");

    nh.param<std::string>("device_port", port, "/dev/ttyUSB0");
    nh.param<std::string>("frame_id", frame_id, "trimble");

    trimbleData_pub = nh.advertise<geometry_msgs::PoseStamped>("data", 1);
    trimbleDataRaw_pub = nh.advertise<geometry_msgs::PointStamped>("dataRaw", 1);

    serial::Serial trimbleSerial(port, SPEED, serial::Timeout::simpleTimeout(100));
    ROS_INFO("serial opened");

    int bytes_avail = 0;
    float tol = 0.001; //tolerance from zero [m]
    float maxSpeed = 10; //maximum speed for vehicle [m/s]
    float x_old = 0.0;
    float y_old = 0.0;
    float z_old = 0.0;
    float dist_old = 0.0;
    ros::Time time_old = ros::Time::now();
    double timeStep = 0.0;
    float speed = 0.0;

    ros::Rate loop_rate(10000);
    while(ros::ok())
    {
        //* *************** *
        //* Get Serial Data *
        //* *************** *
        bytes_avail = trimbleSerial.available();
        if(bytes_avail > 0)
        {
            ros::Time timeStamp0 = ros::Time::now(); //stamp the serial message as soon as it arrives, before parsing
            std::vector<std::string> report_cpp = trimbleSerial.readlines(); //read entire serial buffer until timeout
            int lines = report_cpp.size();
//            ROS_INFO("serial lines = %d",lines);
            float HVr[3]; //total station coordinates (horizontal angle, vertical angle, slope distance)

            //* ***************** *
            //* Parse Serial Data *
            //* ***************** *
            for (int i = 0; i < lines; i++)
            {
                int line1_size = report_cpp[1].length();
                int line2_size = report_cpp[2].length();
                int line3_size = report_cpp[3].length();
                char report_c1[line1_size]; //c string
                char report_c2[line2_size]; //c string
                char report_c3[line3_size]; //c string
//                std::cout << report_cpp[i] << std::endl; //print serial output
                switch(i)
                {
                case 0: //"0/n/n"
                    break;
                case 1: //horz angle
                    strcpy(report_c1,report_cpp[i].c_str()); //change c++ string to c string
                    sscanf(report_c1,"7=%f/n", &HVr[0]);
                    break;
                case 2: //vert angle
                    strcpy(report_c2,report_cpp[i].c_str()); //change c++ string to c string
                    sscanf(report_c2,"8=%f/n", &HVr[1]);
                    break;
                case 3: //slope dist
                    strcpy(report_c3,report_cpp[i].c_str()); //change c++ string to c string
                    sscanf(report_c3,"9=%f/n", &HVr[2]);
                    break;
                case 4: //">/n"
//                    ROS_INFO("horz=%f | vert=%f | dist=%f",HVr[0],HVr[1],HVr[2]); //spherical data
                    break;
                }
            }

            //* ********************** *
            //* Spherical -> Cartesian *
            //* ********************** *

            if (HVr[2] < tol) {HVr[2] = dist_old;} //use previous total station laser measurement if current is zero

            //MATLAB standard spherical variables in standard units
            float r = HVr[2]; // [m]
            float azimuth = (-HVr[0])*M_PI/180.0; // [rad] (axis flipped)
            float elevation = (-HVr[1] + 90)*M_PI/180.0; // [rad] (axis flipped and straight up was zero)


            //spherical to cartesian transformation (https://www.mathworks.com/help/matlab/ref/sph2cart.html)
            float x = r * cos(elevation) * cos(azimuth);
            float y = r * cos(elevation) * sin(azimuth);
            float z = r * sin(elevation);

//            ROS_INFO("x=%f | y=%f | z=%f",x,y,z); //cartesian data


            //* ******* *
            //* Publish *
            //* ******* *

//            //Check how long it took to go from new serial data through calculations
//            ros::Time timeStamp1 = ros::Time::now(); //stamp the message after calcs
//            ros::Duration serial2pub_dur = timeStamp1 - timeStamp0; //delay from data to publish
//            float serial2pub_sec = serial2pub_dur.toSec();
//            ROS_INFO("Time from serial data to publish: %fs", serial2pub_sec);

            pub_pointStamped(HVr[0], HVr[1], HVr[2], timeStamp0); //publish raw data always
            timeStep = (timeStamp0 - time_old).toSec();
            speed = sqrt(pow((x - x_old),2) + pow((y - y_old),2) + pow((z - z_old),2) )/timeStep;
//            ROS_INFO("speed = %.2f",speed);

            if( (abs(x) < tol) && (abs(y) < tol) && (abs(z) < tol) ) //Don't publish zeros
            {
                ROS_INFO("zeros");
            }
            else if (speed > maxSpeed) //don't publish sensor jumps
            {
                ROS_INFO("jump");
            }
            else
            {
                pub_poseStamped(x,y,z,timeStamp0);
                x_old = x; y_old = y; z_old = z; //store data for comparison next message
                time_old = timeStamp0;
                dist_old = HVr[2]; //store slope distance in case it doesn't measure correctly next iteration
            }

        }
        else
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    trimbleSerial.close();
    std::cout << "Closed serial port" << std::endl;
    return 0;
}
