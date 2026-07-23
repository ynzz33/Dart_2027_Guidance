/*
 * File: _coder_LQR_K_Dart_d_mex.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-07-23 18:08:11
 */

/* Include Files */
#include "_coder_LQR_K_Dart_d_mex.h"
#include "_coder_LQR_K_Dart_d_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&LQR_K_Dart_d_atexit);
  LQR_K_Dart_d_initialize();
  unsafe_LQR_K_Dart_d_mexFunction(nlhs, plhs, nrhs, prhs);
  LQR_K_Dart_d_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "GBK", true);
  return emlrtRootTLSGlobal;
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[1]
 * Return Type  : void
 */
void unsafe_LQR_K_Dart_d_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                     int32_T nrhs, const mxArray *prhs[1])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        12, "LQR_K_Dart_d");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "LQR_K_Dart_d");
  }
  /* Call the function. */
  LQR_K_Dart_d_api(prhs[0], &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * File trailer for _coder_LQR_K_Dart_d_mex.c
 *
 * [EOF]
 */
