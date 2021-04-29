/*************************************************
 *  
 *  Class for computing 3D position from Time-Difference of Arrival information.
 *  Position is computed using an Extended Kalman Filter.
 *
 *  
 *
 *  Created on: 06/19/2017
 *  Author: Joao Paulo Jansch Porto <janschp2(at)illinois.edu>
 *
 *  Changelog:
 *      v0.3 - Added extra library functions (06/26/2017)
 *      v0.2 - initial release (06/19/2017)
 *
 *************************************************/

#ifndef _decaDriver_h
#define _decaDriver_h

//#include <cstdio>
//#include <cstdint>
//#include <cstdlib>
//#include <cstring>
//#include <cmath>
//#include <iostream>
//#include <Eigen/Dense>


typedef struct trek1000report_s
{
    char mID;      //(8-bit) message ID, r for raw tag to anchor, c for range bias corrected tag to anchor, a for range bias corrected anchor to anchor ranges
    int  mask;     //(8-bit) valid ranges in binary e.g. 1111 = 0xF is all ranges valid range0 is LSB
    int  range[4]; //(32-bit) ranges are reported in units of millimeters
    int  nRanges;  //(16-bit) this is a number of ranges completed by reporting unit raw range
    int  seq;      //(8-bit) range sequence number
    int  debug;    //(32-bit) if mID = r/c: time of last range reported, if mID = a: TX/RX antenna delays as two 16-bit numbers
    char c;        //(8-bit) report source: a for anchor or t for tag (source ID stays constant for all reports while the communictor ID varies)
    int  aID;      //(8-bit) anchor source/communicator ID
    int  tID;      //(8-bit) tag source/communicator ID (mID = a: always returns 0 for tID)
}trek1000report_t;

//typedef struct rowMajor_s
//{
//    float P [36];
//}rowMajor_t;

//class TDOA
//{
//public:
    
//    // Contructor
//    TDOA();
//    TDOA(const Eigen::MatrixXf transition_mat, const Eigen::MatrixXf prediction_mat, const Eigen::MatrixXf covariance_mat, const vec3d_t init_pos);
    
//    // Set Functions
//    void setTransitionMat(const Eigen::MatrixXf transition_mat);
//    void setPredictionMat(const Eigen::MatrixXf prediction_mat);
//    void setCovarianceMat(const Eigen::MatrixXf covariance_mat);
    
//    void setAncPosition(const int anc_num, const vec3d_t anc_pos);
//    void setAncPosition(const int anc_num, const float x, const float y, const float z);
    
//    void setInitPos(vec3d_t init_pos);
    
//    void setStdDev(float sdev);
    
//    // Update functions
//    void scalarTDOADistUpdate(uint8_t Ar, uint8_t An, float distanceDiff);
//    void stateEstimatorPredict(const double dt);
//    void stateEstimatorFinalize();
//    void stateEstimatorAddProcessNoise();
    
    
//    // Get functions
//    vec3d_t getLocation();
//	vec3d_t getVelocity();
//    vec3d_t getAncPosition(const int anc_num);
//    rowMajor_t getCovariance(); //added by NickW 04/09/2019
    
//private:
    
//    //variables
//    uint32_t tdoaCount;
//    uint32_t nr_states;
//    float stdDev;
    
//    vec3d_t anchorPosition[MAX_NR_ANCHORS];
    
//    // Matrices used by the kalman filter
//    Eigen::VectorXf S;
//    Eigen::MatrixXf P;
//    Eigen::MatrixXf A;
//    Eigen::MatrixXf Q;
//    Eigen::MatrixXf P_new;
    
//    //Functions
//    void stateEstimatorScalarUpdate(Eigen::RowVectorXf H, float error, float stdMeasNoise);
    
//    void PredictionBound();

//};

#endif

