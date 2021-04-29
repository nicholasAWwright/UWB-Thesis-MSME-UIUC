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
#include "ros/package.h"
#include "std_msgs/Header.h"
#include "decawave/trek1000_report.h"

#include "serial/serial.h"
#include "decaSerial.h"

#define SPEED         115200 //serial baud rate

#define TOF_REPORT_LEN  (65) // \r and \n add 2 to length of visible 63
#define TOF_REPORT_ARGS (12)

std::string port, tag_id, frame_id; //ROS params
ros::Publisher decaData_pub; //raw data


void pub_decaData(const trek1000report_t data, const ros::Time timeStamp0)
{
    decawave::trek1000_report serial_msg;

    //Header
    serial_msg.header.seq = 0;
    serial_msg.header.stamp = timeStamp0;
    serial_msg.header.frame_id = frame_id;

    //trek1000_report
    serial_msg.mID = data.mID;
    serial_msg.mask = data.mask;
    serial_msg.range[0] = data.range[0];
    serial_msg.range[1] = data.range[1];
    serial_msg.range[2] = data.range[2];
    serial_msg.range[3] = data.range[3];
    serial_msg.nRanges = data.nRanges;
    serial_msg.seq = data.seq;
    serial_msg.debug = data.debug;
    serial_msg.c = data.c;
    serial_msg.aID = data.aID;
    serial_msg.tID = data.tID;

    decaData_pub.publish(serial_msg);
}


trek1000report_t parseDecaData(const char * s)
{
    trek1000report_t R;
//    R.initialize(); //TODO

    int n; //number of items scanned in report

//    Decawave serial data examples:
//    mr 0f 000005a4 000004c8 00000436 000003f9 0958 c0 40424042 a0:0
//    ma 07 00000000 0000085c 00000659 000006b7 095b 26 00024bed a0:0
//    mc 0f 00000663 000005a3 00000512 000004cb 095f c1 00024c24 a0:0

    n = sscanf(s,"m%c %x %x %x %x %x %x %x %x %c%d:%d", &R.mID,
                                                        &R.mask,
                                                        &R.range[0], &R.range[1], &R.range[2], &R.range[3],
                                                        &R.nRanges,
                                                        &R.seq,
                                                        &R.debug,
                                                        &R.c, &R.tID, &R.aID);

//    //debug print statements
//    std::cout << "number of scanned variables: " << n << std::endl;
//    std::cout << "type(r/c/a):" << R.mID << " T/A:" << R.c << " anc:" << R.aID << " tag:" << R.tID <<  std::endl;
//    std::cout << "range(mm): " << "A0:" << R.range[0] << " A1:" << R.range[1] << " A2:" << R.range [2] << " A3:" << R.range [3] << std::endl;
//    std::cout << "nRanges:"<< R.nRanges << " seq:" << R.seq << " debug:" << R.debug << std::endl;;
//    std::cout << "mask: " << R.mask << std::endl;

    if (n != TOF_REPORT_ARGS)
    {
        ROS_INFO("failed data parsing: tag%s", tag_id.c_str());
//        std::cout << "failed data parsing" << std::endl;
//        R.clear(); //TODO

    }
    return R;
}


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "decaSerial");
    ros::NodeHandle nh("~");

    nh.param<std::string>("device_port", port, "/dev/ttyACM0");
    nh.param<std::string>("tag_id", tag_id, "0");
    nh.param<std::string>("frame_id", frame_id, "uwb_link");

    decaData_pub = nh.advertise<decawave::trek1000_report>("decaData", 1);

    serial::Serial decaSerial(port, SPEED, serial::Timeout::simpleTimeout(1000));
    ROS_INFO("serial opened: tag%s", tag_id.c_str());

    int bytes_avail = 0;
    int count = 0; //debug

    ros::Rate loop_rate(10000);
    while(ros::ok())
    {
        //* *************** *
        //* Get Serial Data *
        //* *************** *
        bytes_avail = decaSerial.available();
        if(bytes_avail > 0)
        {
            std::string tofReport_cpp = decaSerial.readline(); //read entire serial buffer until \n
            ros::Time timeStamp0 = ros::Time::now(); //stamp the serial message as soon as it arrives, before parsing
            if (tofReport_cpp.length() != TOF_REPORT_LEN) //throw away garbage reports (from last ROS run)
            {
//                std::cout << tofReport_cpp << std::endl; //before (garbage)
                ROS_INFO("invalid data: tag%s", tag_id.c_str());
            }
            else
            {
                char tofReport_c[TOF_REPORT_LEN]; //c string
                strcpy(tofReport_c,tofReport_cpp.c_str()); //change c++ string to c string
//                decaData.clear(); //TODO
                trek1000report_t decaData = parseDecaData(tofReport_c); //decaData is a custom structure
                pub_decaData(decaData, timeStamp0);

//                //debug testing to see what load serial publishing can handle. Seems to do fine taking 'a' reports from anchor0 with 4 anchors and 2 tags setup.
//                //also debug testing to find cause of bad data. Looks like adjacent tag channels clog eachother up on TREK1000, at least on SW version 2.10
//                if (decaData.mID != 97 && decaData.mask != 15)
//                {
//                    if (decaData.mask != 7)
//                    {
//                        count++;
//                        ROS_INFO("%d", count);
//                    }
//                }
            }
        }
        else
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    decaSerial.close();
    std::cout << "Closed tag" << tag_id << " serial port" << std::endl;
    return 0;
}
