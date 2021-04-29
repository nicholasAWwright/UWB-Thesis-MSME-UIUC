#include <cstdio>
#include <cstdint>
#include <cstdbool>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <iostream>
#include <limits>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Header.h"
#include "decawave/pdoa_report.h"

#include "serial/serial.h"
#include "pdoaSerial.h"

#define SPEED         115200 //serial baud rate

#define PDOA_REPORT_LEN  (26) // \r and \n add 2 to length of visible 63
#define PDOA_REPORT_ARGS (4)

std::string portHorz, portVert, node_id, frame_id; //ROS params
ros::Publisher decaPDOA_pub; //raw data

void pub_pdoaData(const pdoa_t HorzData, const pdoa_t VertData, const ros::Time timeStamp0)
{
    decawave::pdoa_report pdoa_msg;

    //Header
    pdoa_msg.header.seq = 0;
    pdoa_msg.header.stamp = timeStamp0;
    pdoa_msg.header.frame_id = frame_id;

    //pdoa_report
    pdoa_msg.addrHorz = HorzData.addr;
    pdoa_msg.countHorz = HorzData.count;
    pdoa_msg.distHorz = HorzData.dist;
    pdoa_msg.pdoaHorz = HorzData.pdoa;
    pdoa_msg.addrVert = VertData.addr;
    pdoa_msg.countVert = VertData.count;
    pdoa_msg.distVert = VertData.dist;
    pdoa_msg.pdoaVert = VertData.pdoa;

    decaPDOA_pub.publish(pdoa_msg);
}


pdoa_t parsePDOAdata(const char * s)
{
    pdoa_t R;
//    R.initialize(); //TODO

    int n; //number of items scanned in report

//    Decawave serial data examples:
//    mr 0f 000005a4 000004c8 00000436 000003f9 0958 c0 40424042 a0:0
//    ma 07 00000000 0000085c 00000659 000006b7 095b 26 00024bed a0:0
//    mc 0f 00000663 000005a3 00000512 000004cb 095f c1 00024c24 a0:0

//    n = sscanf(s,"m%c %x %x %x %x %x %x %x %x %c%d:%d", &R.mID,
//                                                        &R.mask,
//                                                        &R.range[0], &R.range[1], &R.range[2], &R.range[3],
//                                                        &R.nRanges,
//                                                        &R.seq,
//                                                        &R.debug,
//                                                        &R.c, &R.tID, &R.aID);


    n = sscanf(s,"AR%04X%02X%08d%08d", &R.addr, &R.count, &R.dist, &R.pdoa);

    //debug print statements
//    std::cout << "number of scanned variables: " << n << std::endl;
    std::cout << "address:" << R.addr << " count:" << R.count << " distance(mm):" << R.dist << " PDoA(mdeg):" << R.pdoa <<  std::endl;

    if (n != PDOA_REPORT_ARGS)
    {
        ROS_INFO("failed data parsing: node_%s", node_id.c_str());
//        std::cout << "failed data parsing" << std::endl;
//        R.clear(); //TODO

    }
    return R;
}


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "pdoaSerial");
    ros::NodeHandle nh("~");

    nh.param<std::string>("device_port_Horz", portHorz, "/dev/ttyACM0");
    nh.param<std::string>("device_port_Vert", portVert, "/dev/ttyACM1");
    nh.param<std::string>("node_id", node_id, "0");
    nh.param<std::string>("frame_id", frame_id, "pdoa");

    decaPDOA_pub = nh.advertise<decawave::pdoa_report>("data", 1);

    //Horizontal PDoA node initialization
    serial::Serial pdoaSerialHorz(portHorz, SPEED, serial::Timeout::simpleTimeout(1000));

    if(pdoaSerialHorz.isOpen())
    {
        ROS_INFO("Horz serial opened");
    }
    else
    {
        ROS_INFO("Horz serial failed to open");
    }

    //sleeps added to ensure that PDoA node has enough time to respond to commands
    pdoaSerialHorz.write("\r\n");
    ros::Duration(0.1).sleep();

    pdoaSerialHorz.write("stop");
    ros::Duration(1.0).sleep();

    pdoaSerialHorz.write("pcrep 3");
    ros::Duration(0.1).sleep();

    ROS_INFO("Horizontal PDoA initialized");

     //Vertical PDoA node initialization
    serial::Serial pdoaSerialVert(portVert, SPEED, serial::Timeout::simpleTimeout(1000));
    if(pdoaSerialVert.isOpen())
    {
        ROS_INFO("Vert serial opened");
    }
    else
    {
        ROS_INFO("Vert serial failed to open");
    }

    //sleeps added to ensure that PDoA node has enough time to respond to commands
    pdoaSerialVert.write("\r\n");
    ros::Duration(0.1).sleep();

    pdoaSerialVert.write("stop");
    ros::Duration(1.0).sleep();

    pdoaSerialVert.write("pcrep 3");
    ros::Duration(0.1).sleep();

    ROS_INFO("Vertical PDoA initialized");


    int dataCount = 205; //number of data points to take from each node before flipping to the other (205, because sometimes 1-2 data points are lost)
    int flagHorz = 1;
    int flagVert = 0;
    int countHorz = 0;
    int countVert = 0;
    int initHorz = 1;
    int initVert = 1;
    pdoa_t storeHorz[dataCount];
    pdoa_t storeVert[dataCount];
    pdoa_t zeros; zeros.addr = 0; zeros.count = 0; zeros.dist = 0; zeros.pdoa = 0; //zeros for publishing

    ros::Rate loop_rate(10000);
    while(ros::ok())
    {
        //* *************** *
        //* Get Serial Data *
        //* *************** *
        // state machine doesn't work with the serial functions
        //HORIZONTAL
        if(flagHorz == 1)
        {
            if(initHorz == 1) //start the sensor for new batch
            {
                pdoaSerialHorz.flush(); //hopefully clear serial buffers
                pdoaSerialHorz.write("node"); //start sensor
                ros::Duration(0.05).sleep(); //wait for hardware
                initHorz = 0;
            }

            int bytes_avail_horz = pdoaSerialHorz.available();

            if(bytes_avail_horz > 0) //if there is data ready
            {
                ros::Time HorzTimeStamp0 = ros::Time::now(); //stamp the serial message as soon as it arrives, before reading
                std::string pdoaReport_cpp = pdoaSerialHorz.readline(); //read entire serial buffer until \n
//                ROS_INFO("%s",pdoaReport_cpp.c_str());

                if (pdoaReport_cpp.length() != PDOA_REPORT_LEN) //throw away unwanted serial info
                {
//                ROS_INFO("unwanted");
//                ROS_INFO("%s",pdoaReport_cpp.c_str());
                }
                else
                {
                    if(pdoaReport_cpp == "error  incompatible mode\r\n") //throw away this error (same length as good data)
                    {
//                    ROS_INFO("error");
//                    ROS_INFO("%s",pdoaReport_cpp.c_str());
                    }
                    else
                    {
                        countHorz++;
                        ROS_INFO("HORZ");
//                        ROS_INFO("%s",pdoaReport_cpp.c_str());
                        char pdoaReport_c[PDOA_REPORT_LEN]; //c string
                        strcpy(pdoaReport_c,pdoaReport_cpp.c_str()); //change c++ string to c string
                        pdoa_t pdoaData = parsePDOAdata(pdoaReport_c); //pdoaData is a custom structure
                        storeHorz[countHorz] = pdoaData; //collect array of pdoa data
//                        pub_pdoaData(pdoaData, HorzTimeStamp0);
                    }

                }
            }
            else if(countHorz >= dataCount)
            {
                //stop horizontal
                pdoaSerialHorz.write("stop");
                ros::Duration(0.05).sleep();

                //calculate

                flagHorz = 0;
                flagVert = 1;
                countVert = 0;
                initVert = 1;


            }

            else
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
        } // end HORIZONTAL

        //VERTICAL
        if(flagVert == 1)
        {
            if(initVert == 1)
            {
                pub_pdoaData(zeros, zeros, ros::Time::now()); //publish line of zeros to start to separate collection locations
                pdoaSerialVert.flush();
                //start vertical
                pdoaSerialVert.write("node");
                ros::Duration(0.05).sleep();
                initVert = 0;
            }
            int bytes_avail_vert = pdoaSerialVert.available();

            if(bytes_avail_vert > 0)
            {
                ros::Time VertTimeStamp0 = ros::Time::now(); //stamp the serial message as soon as it arrives, before reading
                std::string pdoaReport_cpp = pdoaSerialVert.readline(); //read entire serial buffer until \n
//                ROS_INFO("%s",pdoaReport_cpp.c_str());

                if (pdoaReport_cpp.length() != PDOA_REPORT_LEN) //throw away unwanted serial info
                {
//                    ROS_INFO("unwanted");
//                    ROS_INFO("%s",pdoaReport_cpp.c_str());
                }
                else
                {
                    if(pdoaReport_cpp == "error  incompatible mode\r\n") //throw away this error
                    {
//                        ROS_INFO("error");
//                        ROS_INFO("%s",pdoaReport_cpp.c_str());
                    }
                    else
                    {
                        countVert++;
                        ROS_INFO("VERT");
//                        ROS_INFO("%s",pdoaReport_cpp.c_str());
                        char pdoaReport_c[PDOA_REPORT_LEN]; //c string
                        strcpy(pdoaReport_c,pdoaReport_cpp.c_str()); //change c++ string to c string
                        pdoa_t pdoaData = parsePDOAdata(pdoaReport_c); //pdoaData is a custom structure
                        storeVert[countVert] = pdoaData; //collect array of pdoa data
                        pub_pdoaData(storeHorz[countVert], storeVert[countVert], VertTimeStamp0);
                    }
                }
            }
            else if(countVert >= dataCount)
            {
                //stop vertical
                pdoaSerialVert.write("stop");
                ros::Duration(0.05).sleep();

                flagHorz = 0;
                flagVert = 0;
                countHorz = 0;
                initHorz = 1;
            }
            else
            {
                ros::spinOnce();
                loop_rate.sleep();
            }

        } // end VERTICAL

        //Add in a pause in order to move the PDOA tag by hand
        if ((flagHorz == 0) && (flagVert == 0))
        {
            std::cout << "Press ENTER (ONLY ONCE!) to continue... " << std::flush;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n' );

            flagHorz = 1;
            flagVert = 0;

        } // end pause

    } // end while(ros::ok())



    //Close out serial connections
    pdoaSerialHorz.write("pcrep 1"); //return to mode compatible with decawave software
    ros::Duration(0.5).sleep();
    pdoaSerialHorz.write("stop");
    ros::Duration(0.5).sleep();
    pdoaSerialHorz.close();
    std::cout << "Closed horizontal serial port" << std::endl;

    pdoaSerialVert.write("pcrep 1");
    ros::Duration(0.5).sleep();
    pdoaSerialVert.write("stop");
    ros::Duration(0.5).sleep();
    pdoaSerialVert.close();
    std::cout << "Closed vertical serial port" << std::endl;
    return 0;
}
