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

void quad_model_a487e7e2_1_resetStateVector(const void *mech, double *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = 1.0;
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
}

void quad_model_a487e7e2_1_initializeTrackedAngleState(const void *mech, const
  double *motionData, double *state, void *neDiagMgr0)
{
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) motionData;
  (void) state;
}

void quad_model_a487e7e2_1_computeDiscreteState(const void *mech, double *state)
{
  (void) mech;
  (void) state;
}

void quad_model_a487e7e2_1_adjustPosition(const void *mech, const double
  *dofDeltas, double *state)
{
  double xx[14];
  (void) mech;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = dofDeltas[3];
  xx[5] = dofDeltas[4];
  xx[6] = dofDeltas[5];
  pm_math_quatDeriv(xx + 0, xx + 4, xx + 7);
  xx[0] = state[3] + xx[7];
  xx[3] = state[4] + xx[8];
  xx[6] = state[5] + xx[9];
  xx[9] = state[6] + xx[10];
  xx[12] = sqrt(xx[0] * xx[0] + xx[3] * xx[3] + xx[6] * xx[6] + xx[9] * xx[9]);
  xx[13] = 1.0e-64;
  if (xx[13] > xx[12])
    xx[12] = xx[13];
  state[0] = state[0] + dofDeltas[0];
  state[1] = state[1] + dofDeltas[1];
  state[2] = state[2] + dofDeltas[2];
  state[3] = xx[0] / xx[12];
  state[4] = xx[3] / xx[12];
  state[5] = xx[6] / xx[12];
  state[6] = xx[9] / xx[12];
}

static void perturbState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[8] = state[8] - 0.875 * mag;
}

static void perturbState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbState_0_3(double mag, double *state)
{
  double xx[12];
  xx[0] = 0.5 * mag;
  xx[2] = sin(xx[0]);
  xx[1] = fabs(mag);
  xx[3] = 1.0 / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[3]);
  xx[4] = cos(xx[3]);
  xx[7] = sin(2.0 * xx[3]);
  xx[3] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[7] * xx[7]);
  xx[8] = cos(xx[0]);
  xx[9] = xx[2] * xx[1] / xx[3];
  xx[10] = xx[2] * xx[4] / xx[3];
  xx[11] = xx[2] * xx[7] / xx[3];
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  pm_math_quatCompose(xx + 8, xx + 0, xx + 4);
  state[3] = xx[4];
  state[4] = xx[5];
  state[5] = xx[6];
  state[6] = xx[7];
}

static void perturbState_0_3v(double mag, double *state)
{
  double xx[19];
  xx[0] = 0.5 * mag;
  xx[2] = sin(xx[0]);
  xx[3] = fabs(mag);
  xx[5] = 1.0 / (xx[3] - floor(xx[3]) + 1.0e-9);
  xx[3] = sin(xx[5]);
  xx[6] = cos(xx[5]);
  xx[9] = sin(2.0 * xx[5]);
  xx[12] = sqrt(xx[3] * xx[3] + xx[6] * xx[6] + xx[9] * xx[9]);
  xx[15] = cos(xx[0]);
  xx[16] = xx[2] * xx[3] / xx[12];
  xx[17] = xx[2] * xx[6] / xx[12];
  xx[18] = xx[2] * xx[9] / xx[12];
  xx[1] = state[3];
  xx[2] = state[4];
  xx[3] = state[5];
  xx[4] = state[6];
  pm_math_quatCompose(xx + 15, xx + 1, xx + 5);
  state[3] = xx[5];
  state[4] = xx[6];
  state[5] = xx[7];
  state[6] = xx[8];
  state[10] = state[10] + 1.2 * mag;
  state[11] = state[11] - xx[0];
  state[12] = state[12] + 0.9 * mag;
}

void quad_model_a487e7e2_1_perturbState(const void *mech, size_t stageIdx,
  size_t primIdx, double mag, boolean_T doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbState_0_0(mag, state);
    break;

   case 1:
    perturbState_0_0v(mag, state);
    break;

   case 2:
    perturbState_0_1(mag, state);
    break;

   case 3:
    perturbState_0_1v(mag, state);
    break;

   case 4:
    perturbState_0_2(mag, state);
    break;

   case 5:
    perturbState_0_2v(mag, state);
    break;

   case 6:
    perturbState_0_3(mag, state);
    break;

   case 7:
    perturbState_0_3v(mag, state);
    break;
  }
}

void quad_model_a487e7e2_1_computeDofBlendMatrix(const void *mech, size_t
  stageIdx, size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void quad_model_a487e7e2_1_projectPartiallyTargetedPos(const void *mech, size_t
  stageIdx, size_t primIdx, const double *origState, int partialType, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void quad_model_a487e7e2_1_propagateMotion(const void *mech, const double *state,
  double *motionData)
{
  double xx[12];
  (void) mech;
  xx[0] = - state[3];
  xx[1] = - state[4];
  xx[2] = - state[5];
  xx[3] = - state[6];
  xx[4] = 5.0;
  xx[6] = state[7];
  xx[7] = state[8];
  xx[8] = state[9];
  pm_math_quatInverseXform(xx + 0, xx + 6, xx + 9);
  motionData[0] = xx[0];
  motionData[1] = xx[1];
  motionData[2] = xx[2];
  motionData[3] = xx[3];
  motionData[4] = state[0] + xx[4];
  motionData[5] = state[1];
  motionData[6] = state[2] + xx[4];
  motionData[7] = state[10];
  motionData[8] = state[11];
  motionData[9] = state[12];
  motionData[10] = xx[9];
  motionData[11] = xx[10];
  motionData[12] = xx[11];
}

size_t quad_model_a487e7e2_1_computeAssemblyError(const void *mech, size_t
  constraintIdx, const double *state, const double *motionData, double *error)
{
  (void) mech;
  (void) state;
  (void) motionData;
  (void) error;
  (void) state;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t quad_model_a487e7e2_1_computeAssemblyJacobian(const void *mech, size_t
  constraintIdx, boolean_T forVelocitySatisfaction, const double *state, const
  double *motionData, double *J)
{
  (void) mech;
  (void) state;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t quad_model_a487e7e2_1_computeFullAssemblyJacobian(const void *mech, const
  double *state, const double *motionData, double *J)
{
  (void) mech;
  (void) state;
  (void) motionData;
  (void) J;
  return 0;
}

int quad_model_a487e7e2_1_isInKinematicSingularity(const void *mech, size_t
  constraintIdx, const double *motionData)
{
  (void) mech;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

PmfMessageId quad_model_a487e7e2_1_convertStateVector(const void *asmMech, const
  void *simMech, const double *asmState, double *simState, void *neDiagMgr0)
{
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) asmMech;
  (void) simMech;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  return NULL;
}

void quad_model_a487e7e2_1_constructStateVector(const void *mech, const double
  *solverState, const double *u, const double *uDot, double *discreteState,
  double *fullState)
{
  (void) mech;
  (void) u;
  (void) uDot;
  (void) discreteState;
  fullState[0] = solverState[0];
  fullState[1] = solverState[1];
  fullState[2] = solverState[2];
  fullState[3] = solverState[3];
  fullState[4] = solverState[4];
  fullState[5] = solverState[5];
  fullState[6] = solverState[6];
  fullState[7] = solverState[7];
  fullState[8] = solverState[8];
  fullState[9] = solverState[9];
  fullState[10] = solverState[10];
  fullState[11] = solverState[11];
  fullState[12] = solverState[12];
}

void quad_model_a487e7e2_1_extractSolverStateVector(const void *mech, const
  double *fullState, double *solverState)
{
  (void) mech;
  solverState[0] = fullState[0];
  solverState[1] = fullState[1];
  solverState[2] = fullState[2];
  solverState[3] = fullState[3];
  solverState[4] = fullState[4];
  solverState[5] = fullState[5];
  solverState[6] = fullState[6];
  solverState[7] = fullState[7];
  solverState[8] = fullState[8];
  solverState[9] = fullState[9];
  solverState[10] = fullState[10];
  solverState[11] = fullState[11];
  solverState[12] = fullState[12];
}

int quad_model_a487e7e2_1_isPositionViolation(const void *mech, const double
  *state)
{
  (void) mech;
  (void) state;
  return 0;
}

int quad_model_a487e7e2_1_isVelocityViolation(const void *mech, const double
  *state)
{
  (void) mech;
  (void) state;
  return 0;
}

PmfMessageId quad_model_a487e7e2_1_projectStateSim(const void *mech, const
  double *input, double *state, void *neDiagMgr0)
{
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  double xx[1];
  (void) mech;
  (void) input;
  xx[0] = 1.0 / sqrt(state[3] * state[3] + state[4] * state[4] + state[5] *
                     state[5] + state[6] * state[6]);
  state[3] = state[3] * xx[0];
  state[4] = state[4] * xx[0];
  state[5] = state[5] * xx[0];
  state[6] = state[6] * xx[0];
  return NULL;
}
