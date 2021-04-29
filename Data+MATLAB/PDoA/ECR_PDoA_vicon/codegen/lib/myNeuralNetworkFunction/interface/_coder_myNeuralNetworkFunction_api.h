/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_myNeuralNetworkFunction_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 13-May-2020 20:31:24
 */

#ifndef _CODER_MYNEURALNETWORKFUNCTION_API_H
#define _CODER_MYNEURALNETWORKFUNCTION_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void myNeuralNetworkFunction(real_T x1, real_T b_y1[3]);
extern void myNeuralNetworkFunction_api(const mxArray * const prhs[1], int32_T
  nlhs, const mxArray *plhs[1]);
extern void myNeuralNetworkFunction_atexit(void);
extern void myNeuralNetworkFunction_initialize(void);
extern void myNeuralNetworkFunction_terminate(void);
extern void myNeuralNetworkFunction_xil_shutdown(void);
extern void myNeuralNetworkFunction_xil_terminate(void);

#endif

/*
 * File trailer for _coder_myNeuralNetworkFunction_api.h
 *
 * [EOF]
 */
