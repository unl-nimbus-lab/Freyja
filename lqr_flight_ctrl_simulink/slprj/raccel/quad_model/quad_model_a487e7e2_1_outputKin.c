/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'quad_model/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"

PmfMessageId quad_model_a487e7e2_1_outputKin(const double *state, const double
  *input, const double *inputDot, const double *inputDdot, const double
  *discreteState, double *output, NeuDiagnosticManager *neDiagMgr)
{
  double xx[17];
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 0.0;
  xx[1] = state[3];
  xx[2] = state[4];
  xx[3] = state[5];
  xx[4] = state[6];
  xx[5] = state[10];
  xx[6] = state[11];
  xx[7] = state[12];
  pm_math_quatXform(xx + 1, xx + 5, xx + 8);
  xx[1] = 5.0;
  xx[3] = - state[3];
  xx[4] = - state[4];
  xx[5] = - state[5];
  xx[6] = - state[6];
  xx[11] = state[7];
  xx[12] = state[8];
  xx[13] = state[9];
  pm_math_quatInverseXform(xx + 3, xx + 11, xx + 14);
  pm_math_quatXform(xx + 3, xx + 14, xx + 11);
  output[0] = state[0];
  output[1] = state[7];
  output[3] = state[1];
  output[4] = state[8];
  output[6] = state[2];
  output[7] = state[9];
  output[9] = state[3];
  output[10] = state[4];
  output[11] = state[5];
  output[12] = state[6];
  output[13] = xx[8];
  output[14] = xx[9];
  output[15] = xx[10];
  output[19] = state[3];
  output[20] = state[4];
  output[21] = state[5];
  output[22] = state[6];
  output[23] = state[0] + xx[1];
  output[24] = state[1];
  output[25] = state[2] + xx[1];
  output[26] = xx[11];
  output[27] = xx[12];
  output[28] = xx[13];
  return NULL;
}
