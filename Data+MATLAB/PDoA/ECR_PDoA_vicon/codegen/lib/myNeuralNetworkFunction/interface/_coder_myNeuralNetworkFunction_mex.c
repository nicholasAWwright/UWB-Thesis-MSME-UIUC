/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_myNeuralNetworkFunction_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 13-May-2020 20:31:24
 */

/* Include Files */
#include "_coder_myNeuralNetworkFunction_mex.h"
#include "_coder_myNeuralNetworkFunction_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void c_myNeuralNetworkFunction_mexFu(int32_T nlhs, mxArray
  *plhs[1], int32_T nrhs, const mxArray *prhs[1]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[1]
 * Return Type  : void
 */
void c_myNeuralNetworkFunction_mexFu(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[1])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        23, "myNeuralNetworkFunction");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 23,
                        "myNeuralNetworkFunction");
  }

  /* Call the function. */
  myNeuralNetworkFunction_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(&myNeuralNetworkFunction_atexit);

  /* Module initialization. */
  myNeuralNetworkFunction_initialize();

  /* Dispatch the entry-point. */
  c_myNeuralNetworkFunction_mexFu(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  myNeuralNetworkFunction_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_myNeuralNetworkFunction_mex.c
 *
 * [EOF]
 */
