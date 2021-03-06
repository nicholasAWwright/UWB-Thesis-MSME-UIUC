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

#ifndef _TDOA_h
#define _TDOA_h

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#define STATE_X   0
#define STATE_Y   1
#define STATE_Z   2
#define STATE_VX  3
#define STATE_VY  4
#define STATE_VZ  5
#define STATE_DIM 6

#define MAX_NR_ANCHORS 8

#define MAX_COVARIANCE 100
#define MIN_COVARIANCE 1e-6f

typedef struct vec3d_s
{
    float   x;
    float   y;
    float   z;
}vec3d_t;

typedef struct rowMajor_s
{
    float P [36];
}rowMajor_t;

class TDOA
{
public:
    
    // Contructor
    TDOA();
    TDOA(const Eigen::MatrixXf transition_mat, const Eigen::MatrixXf prediction_mat, const Eigen::MatrixXf covariance_mat, const vec3d_t init_pos);
    
    // Set Functions
    void setTransitionMat(const Eigen::MatrixXf transition_mat);
    void setPredictionMat(const Eigen::MatrixXf prediction_mat);
    void setCovarianceMat(const Eigen::MatrixXf covariance_mat);
    
    void setAncPosition(const int anc_num, const vec3d_t anc_pos);
    void setAncPosition(const int anc_num, const float x, const float y, const float z);
    
    void setInitPos(vec3d_t init_pos);
    
    void setStdDev(float sdev);
    
    // Update functions
    void scalarTDOADistUpdate(uint8_t Ar, uint8_t An, float distanceDiff);
    void stateEstimatorPredict(const double dt);
    void stateEstimatorFinalize();
    void stateEstimatorAddProcessNoise();
    
    
    // Get functions
    vec3d_t getLocation();
	vec3d_t getVelocity();
    vec3d_t getAncPosition(const int anc_num);
    rowMajor_t getCovariance(); //added by NickW 04/09/2019
    
private:
    
    //variables
    uint32_t tdoaCount;
    uint32_t nr_states;
    float stdDev;
    
    vec3d_t anchorPosition[MAX_NR_ANCHORS];
    
    // Matrices used by the kalman filter
    Eigen::VectorXf S;
    Eigen::MatrixXf P;
    Eigen::MatrixXf A;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf P_new;
    
    //Functions
    void stateEstimatorScalarUpdate(Eigen::RowVectorXf H, float error, float stdMeasNoise);
    
    void PredictionBound();

};

#endif

