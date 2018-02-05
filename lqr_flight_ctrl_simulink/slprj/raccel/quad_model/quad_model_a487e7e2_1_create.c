/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'quad_model/Solver Configuration'.
 */

#include "pm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "pm_default_allocator.h"
#include "sm_ssci_NeDaePrivateData.h"
#include "sm_CTarget.h"

PmfMessageId sm_ssci_recordRunTimeError(
  const char *errorId, const char *errorMsg, NeuDiagnosticManager* mgr);

#define pm_allocator_alloc(_allocator, _m, _n) ((_allocator)->mCallocFcn((_allocator), (_m), (_n)))
#define PM_ALLOCATE_ARRAY(_name, _type, _size, _allocator)\
 _name = (_type *) pm_allocator_alloc(_allocator, sizeof(_type), _size)
#define pm_size_to_int(_size)          ((int32_T) (_size))

PmIntVector *pm_create_int_vector(size_t, PmAllocator *);
int_T pm_create_int_vector_fields (PmIntVector *, size_t, PmAllocator *);
int_T pm_create_real_vector_fields(PmRealVector *, size_t, PmAllocator *);
int_T pm_create_char_vector_fields(PmCharVector *, size_t, PmAllocator *);
int_T pm_create_bool_vector_fields(PmBoolVector *, size_t, PmAllocator *);
void pm_rv_equals_rv(const PmRealVector *, const PmRealVector *);
void sm_ssci_setupLoggerFcn_codeGen(const NeDae *dae,
  NeLoggerBuilder *neLoggerBuilder);
int32_T sm_ssci_logFcn_codeGen(const NeDae *dae,
  const NeSystemInput *systemInput,
  PmRealVector *output);
PmfMessageId quad_model_a487e7e2_1_deriv(
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId quad_model_a487e7e2_1_checkDynamics(
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId quad_model_a487e7e2_1_outputDyn(
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId quad_model_a487e7e2_1_outputKin(
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId quad_model_a487e7e2_1_output (
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
void quad_model_a487e7e2_1_resetStateVector(const void *mech, double
  *stateVector);
void quad_model_a487e7e2_1_initializeTrackedAngleState(
  const void *mech,
  const double *motionData,
  double *stateVector,
  void *neDiagMgr);
void quad_model_a487e7e2_1_computeDiscreteState(const void *mech, double
  *stateVector);
void quad_model_a487e7e2_1_adjustPosition(
  const void *mech,
  const double *dofDeltas,
  double *stateVector);
void quad_model_a487e7e2_1_perturbState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void quad_model_a487e7e2_1_computeDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void quad_model_a487e7e2_1_projectPartiallyTargetedPos(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *origStateVector,
  int partialType,
  double *stateVector);
void quad_model_a487e7e2_1_propagateMotion(
  const void *mech,
  const double *stateVector,
  double *motionData);
size_t quad_model_a487e7e2_1_computeAssemblyError(
  const void *mech,
  size_t constraintIdx,
  const double *stateVector,
  const double *motionData,
  double *error);
size_t quad_model_a487e7e2_1_computeAssemblyJacobian(
  const void *mech,
  size_t constraintIdx,
  boolean_T forVelocitySatisfaction,
  const double *stateVector,
  const double *motionData,
  double *J);
size_t quad_model_a487e7e2_1_computeFullAssemblyJacobian(
  const void *mech,
  const double *stateVector,
  const double *motionData,
  double *J);
int quad_model_a487e7e2_1_isInKinematicSingularity(
  const void *mech,
  size_t constraintIdx,
  const double *motionData);
PmfMessageId quad_model_a487e7e2_1_convertStateVector(
  const void *asmMech,
  const void *simMech,
  const double *asmStateVector,
  double *simStateVector,
  void *neDiagMgr);
void quad_model_a487e7e2_1_constructStateVector(
  const void *mech,
  const double *solverStateVector,
  const double *u,
  const double *uDot,
  const double *discreteStateVector,
  double *fullStateVector);
void quad_model_a487e7e2_1_extractSolverStateVector(
  const void *mech,
  const double *fullStateVector,
  double *solverStateVector);
int quad_model_a487e7e2_1_isPositionViolation(
  const void *mech,
  const double *stateVector);
int quad_model_a487e7e2_1_isVelocityViolation(
  const void *mech,
  const double *stateVector);
PmfMessageId quad_model_a487e7e2_1_projectStateSim(
  const void *mech,
  const double *inputVector,
  double *stateVector,
  void *neDiagMgr);
PmfMessageId quad_model_a487e7e2_1_assemble(const double *u, double *udot,
  double *x,
  NeuDiagnosticManager *neDiagMgr)
{
  (void) x;
  (void) u;
  (void) udot;
  return NULL;
}

static
  PmfMessageId dae_cg_deriv_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  if (smData->mCachedDerivativesAvailable)
    memcpy(daeMethodOutput->mXP0.mX, smData->mCachedDerivatives.mX,
           13 * sizeof(real_T));
  else
    errorId = quad_model_a487e7e2_1_deriv(
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 6,
      systemInput->mV.mX + 6,
      systemInput->mD.mX,
      daeMethodOutput->mXP0.mX,
      &errorResult,
      neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_output_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  PmfMessageId errorId = NULL;
  NeDaePrivateData *smData = dae->mPrivateData;
  if (smData->mDoComputeDynamicOutputs) {
    int derivErr = 0;
    double errorResult = 0.0;
    errorId = quad_model_a487e7e2_1_outputDyn(
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 6,
      systemInput->mV.mX + 6,
      systemInput->mD.mX,
      smData->mCachedDerivatives.mX, daeMethodOutput->mY.mX,
      &derivErr, &errorResult, neDiagMgr);
    smData->mCachedDerivativesAvailable = (derivErr == 0);
  } else
    errorId = quad_model_a487e7e2_1_outputKin(
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 6,
      systemInput->mV.mX + 6,
      systemInput->mD.mX,
      daeMethodOutput->mY.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_project_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  return
    sm_core_projectState(
    false,
    &smData->mSimulationDelegate,
    systemInput->mU.mX,
    systemInput->mU.mX + 6,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
}

static
  PmfMessageId dae_cg_check_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  if (smData->mNumConstraintEqns > 0)
    errorId = sm_core_projectState(
      false,
      &smData->mSimulationDelegate,
      systemInput->mU.mX,
      systemInput->mU.mX + 6,
      systemInput->mD.mX,
      systemInput->mX.mX, neDiagMgr);
  if (errorId == NULL && smData->mDoCheckDynamics) {
    double result = 0.0;
    errorId = quad_model_a487e7e2_1_checkDynamics(
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 6,
      systemInput->mV.mX + 6,
      systemInput->mD.mX,
      &result, neDiagMgr);
  }

  return errorId;
}

static
  PmfMessageId dae_cg_projectMaybe_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  return
    sm_core_projectState(
    true,
    &smData->mSimulationDelegate,
    systemInput->mU.mX,
    systemInput->mU.mX + 6,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
}

static
  PmfMessageId dae_cg_assemble_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  if (smData->mNumInputMotionPrimitives == 0) {
    pm_rv_equals_rv(&systemInput->mX, &dae->mPrivateData->mInitialStateVector);
    pm_rv_equals_rv(&systemInput->mD, &dae->mPrivateData->mDiscreteStateVector);
  } else {
    size_t i;
    const size_t numTargets = 8;
    unsigned int asmStatus = 0;
    double *assemblyFullStateVector = smData->mAssemblyFullStateVector.mX;
    double *simulationFullStateVector = smData->mSimulationFullStateVector.mX;
    const double *u = systemInput->mU.mX;
    const double *uDot = u + smData->mInputVectorSize;
    CTarget *target = smData->mTargets + smData->mNumInternalTargets;
    for (i = 0; i < smData->mNumInputMotionPrimitives; ++i) {
      const size_t inputOffset = smData->mMotionInputOffsets.mX[i];
      (target++)->mValue[0] = u [inputOffset];
      (target++)->mValue[0] = uDot[inputOffset];
    }

    errorId = sm_core_computeStateVector(
      &smData->mMechanismDelegate, numTargets, smData->mTargets,
      assemblyFullStateVector, neDiagMgr);
    if (errorId != NULL)
      return errorId;
    asmStatus = sm_core_checkAssembly(
      &smData->mMechanismDelegate, numTargets, smData->mTargets,
      assemblyFullStateVector, NULL, NULL, NULL);
    if (asmStatus != 1) {
      return sm_ssci_recordRunTimeError(
        "sm:compiler:messages:simulationErrors:AssemblyFailure",
        asmStatus == 2 ?
        "Model not assembled due to a position violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
        :
        (asmStatus == 3 ?
         "Model not assembled due to a velocity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
         :
         "Model not assembled due to a singularity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."),
        neDiagMgr);
    }

    errorId =
      (*smData->mMechanismDelegate.mConvertStateVector)(
      NULL, NULL, assemblyFullStateVector, simulationFullStateVector,
      neDiagMgr);
    for (i = 0; i < smData->mStateVectorSize; ++i)
      systemInput->mX.mX[i] = simulationFullStateVector
        [smData->mStateVectorMap.mX[i]];
    memcpy(systemInput->mD.mX,
           simulationFullStateVector +
           smData->mFullStateVectorSize - smData->mDiscreteStateSize,
           smData->mDiscreteStateSize * sizeof(double));
  }

  return errorId;
}

typedef struct {
  size_t first;
  size_t second;
} SizePair;

static void checkMemAllocStatus(int_T status)
{
  (void) status;
}

static
  PmCharVector cStringToCharVector(const char *src)
{
  const size_t n = strlen(src);
  PmCharVector charVect;
  const int_T status =
    pm_create_char_vector_fields(&charVect, n + 1, pm_default_allocator());
  checkMemAllocStatus(status);
  strcpy(charVect.mX, src);
  return charVect;
}

static
  void initBasicAttributes(NeDaePrivateData *smData)
{
  size_t i;
  smData->mStateVectorSize = 13;
  smData->mFullStateVectorSize = 13;
  smData->mDiscreteStateSize = 0;
  smData->mInputVectorSize = 6;
  smData->mOutputVectorSize = 29;
  smData->mNumConstraintEqns = 0;
  smData->mDoCheckDynamics = false;
  for (i = 0; i < 4; ++i)
    smData->mChecksum[i] = 0;
}

static
  void initStateVector(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const double initialStateVector[13] = {
    +0.000000000000000000e+00, +0.000000000000000000e+00,
    +0.000000000000000000e+00, +1.000000000000000000e+00,
    +0.000000000000000000e+00, +0.000000000000000000e+00,
    +0.000000000000000000e+00, +0.000000000000000000e+00,
    +0.000000000000000000e+00, +0.000000000000000000e+00,
    +0.000000000000000000e+00, +0.000000000000000000e+00,
    +0.000000000000000000e+00
  };

  const double *discreteStateVector = NULL;
  const int32_T stateVectorMap[13] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12
  };

  const CTarget targets[8] = {
    { 0, 7, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 7, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true, 1,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 7, 1, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 7, 1, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true, 1,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 7, 2, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 7, 2, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true, 1,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 7, 3, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 7, 3, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true, 3,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } }
  };

  const size_t numTargets = 8;
  int_T status;
  size_t i;
  status = pm_create_real_vector_fields(
    &smData->mAssemblyFullStateVector, 13, alloc);
  checkMemAllocStatus(status);
  status = pm_create_real_vector_fields(
    &smData->mSimulationFullStateVector, 13, alloc);
  checkMemAllocStatus(status);
  status = pm_create_real_vector_fields(
    &smData->mInitialStateVector, 13, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mInitialStateVector.mX, initialStateVector,
         13 * sizeof(real_T));
  status = pm_create_real_vector_fields(
    &smData->mDiscreteStateVector, 0, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mDiscreteStateVector.mX, discreteStateVector,
         0 * sizeof(real_T));
  status = pm_create_int_vector_fields(
    &smData->mStateVectorMap, smData->mStateVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mStateVectorMap.mX, stateVectorMap,
         smData->mStateVectorSize * sizeof(int32_T));
  smData->mNumInternalTargets = 8;
  smData->mNumInputMotionPrimitives = 0;
  PM_ALLOCATE_ARRAY(smData->mTargets, CTarget, numTargets, alloc);
  for (i = 0; i < numTargets; ++i)
    sm_compiler_CTarget_copy(targets + i, smData->mTargets + i);
}

static
  void initVariables(NeDaePrivateData *smData)
{
  const char *varFullPaths[13] = {
    "HUMMINGBIRD.Px.p",
    "HUMMINGBIRD.Py.p",
    "HUMMINGBIRD.Pz.p",
    "HUMMINGBIRD.S.Q",
    "HUMMINGBIRD.S.Q",
    "HUMMINGBIRD.S.Q",
    "HUMMINGBIRD.S.Q",
    "HUMMINGBIRD.Px.v",
    "HUMMINGBIRD.Py.v",
    "HUMMINGBIRD.Pz.v",
    "HUMMINGBIRD.S.w",
    "HUMMINGBIRD.S.w",
    "HUMMINGBIRD.S.w"
  };

  const char *varObjects[13] = {
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD",
    "quad_model/HUMMINGBIRD"
  };

  smData->mNumVarScalars = 13;
  smData->mVarFullPaths = NULL;
  smData->mVarObjects = NULL;
  if (smData->mNumVarScalars > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(smData->mVarFullPaths, PmCharVector, 13, alloc);
    PM_ALLOCATE_ARRAY(smData->mVarObjects, PmCharVector, 13, alloc);
    for (s = 0; s < smData->mNumVarScalars; ++s) {
      smData->mVarFullPaths[s] = cStringToCharVector(varFullPaths[s]);
      smData->mVarObjects[s] = cStringToCharVector(varObjects[s]);
    }
  }
}

static
  void initIoInfoHelper(
  size_t n,
  const char *portPathsSource[],
  const char *unitsSource[],
  const SizePair dimensions[],
  boolean_T doInputs,
  NeDaePrivateData *smData)
{
  PmCharVector *portPaths = NULL;
  PmCharVector *units = NULL;
  NeDsIoInfo *infos = NULL;
  if (n > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(portPaths, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(units, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(infos, NeDsIoInfo, n, alloc);
    for (s = 0; s < n; ++s) {
      portPaths[s] = cStringToCharVector(portPathsSource[s]);
      units[s] = cStringToCharVector(unitsSource[s]);

      {
        NeDsIoInfo *info = infos + s;
        info->mName = info->mIdentifier = portPaths[s].mX;
        info->mM = dimensions[s].first;
        info->mN = dimensions[s].second;
        info->mUnit = units[s].mX;
      }
    }
  }

  if (doInputs) {
    smData->mNumInputs = n;
    smData->mInputPortPaths = portPaths;
    smData->mInputUnits = units;
    smData->mInputInfos = infos;
  } else {
    smData->mNumOutputs = n;
    smData->mOutputPortPaths = portPaths;
    smData->mOutputUnits = units;
    smData->mOutputInfos = infos;
  }
}

static
  void initIoInfo(NeDaePrivateData *smData)
{
  const char *inputPortPaths[6] = {
    "HUMMINGBIRD.fxi",
    "HUMMINGBIRD.fyi",
    "HUMMINGBIRD.fzi",
    "HUMMINGBIRD.txi",
    "HUMMINGBIRD.tyi",
    "HUMMINGBIRD.tzi"
  };

  const char *inputUnits[6] = {
    "m*kg/s^2",
    "m*kg/s^2",
    "m*kg/s^2",
    "m^2*kg/s^2",
    "m^2*kg/s^2",
    "m^2*kg/s^2"
  };

  const SizePair inputDimensions[6] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }
  };

  const char *outputPortPaths[23] = {
    "HUMMINGBIRD.px",
    "HUMMINGBIRD.vx",
    "HUMMINGBIRD.ax",
    "HUMMINGBIRD.py",
    "HUMMINGBIRD.vy",
    "HUMMINGBIRD.ay",
    "HUMMINGBIRD.pz",
    "HUMMINGBIRD.vz",
    "HUMMINGBIRD.az",
    "HUMMINGBIRD.Q",
    "HUMMINGBIRD.wx",
    "HUMMINGBIRD.wy",
    "HUMMINGBIRD.wz",
    "HUMMINGBIRD.bx",
    "HUMMINGBIRD.by",
    "HUMMINGBIRD.bz",
    "Transform_Sensor.Q",
    "Transform_Sensor.x",
    "Transform_Sensor.y",
    "Transform_Sensor.z",
    "Transform_Sensor.vx",
    "Transform_Sensor.vy",
    "Transform_Sensor.vz"
  };

  const char *outputUnits[23] = {
    "m",
    "m/s",
    "m/s^2",
    "m",
    "m/s",
    "m/s^2",
    "m",
    "m/s",
    "m/s^2",
    "1",
    "rad/s",
    "rad/s",
    "rad/s",
    "rad/s^2",
    "rad/s^2",
    "rad/s^2",
    "1",
    "m",
    "m",
    "m",
    "m/s",
    "m/s",
    "m/s"
  };

  const SizePair outputDimensions[23] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 4, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 4, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }
  };

  initIoInfoHelper(6, inputPortPaths, inputUnits, inputDimensions,
                   true, smData);
  initIoInfoHelper(23, outputPortPaths, outputUnits, outputDimensions,
                   false, smData);
}

static
  void initInputDerivs(NeDaePrivateData *smData)
{
  const int32_T numInputDerivs[6] = {
    0, 0, 0, 0, 0, 0
  };

  PmAllocator *alloc = pm_default_allocator();
  const int_T status = pm_create_int_vector_fields(
    &smData->mNumInputDerivs, smData->mInputVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mNumInputDerivs.mX, numInputDerivs,
         6 * sizeof(int32_T));
  smData->mInputOrder = 1;
}

static
  void initDirectFeedthrough(NeDaePrivateData *smData)
{
  const boolean_T directFeedthroughVector[6] = {
    false, false, false, false, false, false
  };

  const boolean_T directFeedthroughMatrix[174] = {
    false, false, true, false, false, true, false, false, true, false,
    false, false, false, false, false, false, true, true, true, false,
    false, false, false, false, false, false, false, false, false, false,
    false, true, false, false, true, false, false, true, false, false,
    false, false, false, false, false, true, true, true, false, false,
    false, false, false, false, false, false, false, false, false, false,
    true, false, false, true, false, false, true, false, false, false,
    false, false, false, false, true, true, true, false, false, false,
    false, false, false, false, false, false, false, false, false, true,
    false, false, true, false, false, true, false, false, false, false,
    false, false, false, true, true, true, false, false, false, false,
    false, false, false, false, false, false, false, false, true, false,
    false, true, false, false, true, false, false, false, false, false,
    false, false, true, true, true, false, false, false, false, false,
    false, false, false, false, false, false, false, true, false, false,
    true, false, false, true, false, false, false, false, false, false,
    false, true, true, true, false, false, false, false, false, false,
    false, false, false, false
  };

  PmAllocator *alloc = pm_default_allocator();

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughVector, 6, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughVector.mX, directFeedthroughVector,
           6 * sizeof(boolean_T));
  }

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughMatrix, 174, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughMatrix.mX, directFeedthroughMatrix,
           174 * sizeof(boolean_T));
  }
}

static
  void initOutputDerivProc(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T outputFunctionMap[29] = {
    0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0
  };

  smData->mOutputFunctionMap = pm_create_int_vector(29, alloc);
  memcpy(smData->mOutputFunctionMap->mX, outputFunctionMap,
         29 * sizeof(int32_T));
  smData->mNumOutputClasses = 2;
  smData->mHasKinematicOutputs = true;
  smData->mHasDynamicOutputs = true;
  smData->mIsOutputClass0Dynamic = false;
  smData->mDoComputeDynamicOutputs = false;
  smData->mCachedDerivativesAvailable = false;

  {
    size_t i = 0;
    const int_T status = pm_create_real_vector_fields(
      &smData->mCachedDerivatives, 13, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mCachedDerivatives.mN; ++i)
      smData->mCachedDerivatives.mX[i] = 0.0;
  }
}

#if 0

static void initializeSizePairVector(const SmSizePair *data,
  SmSizePairVector *vector)
{
  const size_t n = sm_core_SmSizePairVector_size(vector);
  size_t i;
  for (i = 0; i < n; ++i, ++data)
    sm_core_SmSizePairVector_setValue(vector, i, data++);
}

#endif

static
  void initMechanismDelegate(SmMechanismDelegate *delegate)
{
  PmAllocator *alloc = pm_default_allocator();
  const SmSizePair jointToStageIdx[1] = {
    { 7, 0 }
  };

  const size_t primitiveIndices[1 + 1] = {
    0, 4
  };

  const SmSizePair stateOffsets[4] = {
    { 0, 7 }, { 1, 8 }, { 2, 9 }, { 3, 10 }
  };

  const SmSizePair dofOffsets[4] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 6 }
  };

  const size_t *remodIndices = NULL;
  const size_t *equationsPerConstraint = NULL;
  const size_t dofToVelSlot[6] = {
    7, 8, 9, 10, 11, 12
  };

  const size_t *constraintDofs = NULL;
  const size_t constraintDofOffsets[0 + 1] = {
    0
  };

  const size_t Jm = 0;
  const size_t Jn = 6;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000062e-09;
  delegate->mDof = 6;
  delegate->mStateSize = 13;
  delegate->mNumStages = 1;
  delegate->mNumConstraints = 0;
  delegate->mNumAllConstraintEquations = 0;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, delegate->mNumStages, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, delegate->mNumStages * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 4, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 4 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 4, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 4 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&delegate->mPosRequired, 6);
  sm_core_SmBoundedSet_create(&delegate->mPosDesired, 6);
  sm_core_SmBoundedSet_create(&delegate->mPosSuggested, 6);
  sm_core_SmBoundedSet_create(&delegate->mPosFree, 6);
  sm_core_SmBoundedSet_create(&delegate->mPosNonRequired, 6);
  sm_core_SmBoundedSet_create(&delegate->mPosSuggAndFree, 6);
  sm_core_SmBoundedSet_create(&delegate->mVelRequired, 6);
  sm_core_SmBoundedSet_create(&delegate->mVelDesired, 6);
  sm_core_SmBoundedSet_create(&delegate->mVelSuggested, 6);
  sm_core_SmBoundedSet_create(&delegate->mVelFree, 6);
  sm_core_SmBoundedSet_create(&delegate->mVelNonRequired, 6);
  sm_core_SmBoundedSet_create(&delegate->mVelSuggAndFree, 6);
  sm_core_SmBoundedSet_create(&delegate->mConstraintFilter, 0);
  sm_core_SmBoundedSet_create(&delegate->mActiveConstraints, 0);
  sm_core_SmBoundedSet_create(&delegate->mActiveDofs, 6);
  sm_core_SmBoundedSet_create(&delegate->mActiveDofs0, 6);
  sm_core_SmBoundedSet_create(&delegate->mNewConstraints, 0);
  sm_core_SmBoundedSet_create(&delegate->mNewDofs, 6);
  sm_core_SmBoundedSet_create(&delegate->mUnsatisfiedConstraints, 0);
  sm_core_SmSizeTVector_create(&delegate->mActiveConstraintsVect,
    0, 0);
  sm_core_SmSizeTVector_create(&delegate->mActiveDofsVect, 6, 0);
  sm_core_SmSizeTVector_create(&delegate->mFullDofToActiveDof, 6, 0);
  sm_core_SmSizePairVector_create(
    &delegate->mPartiallyPosTargetedPrims, 4, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &delegate->mPartiallyVelTargetedPrims, 4, &zeroSizePair);
  sm_core_SmSizeTVector_create(&delegate->mPosPartialTypes, 4, 0);
  sm_core_SmSizeTVector_create(&delegate->mVelPartialTypes, 4, 0);
  sm_core_SmSizeTVector_create(&delegate->mPartiallyActivePrims, 4, 0);
  sm_core_SmSizePairVector_create(
    &delegate->mBaseFrameVelOffsets, 1, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &delegate->mCvVelOffsets, 4, &zeroSizePair);
  sm_core_SmRealVector_create(&delegate->mInitialState, 13, 0.0);
  sm_core_SmRealVector_create(&delegate->mStartState, 13, 0.0);
  sm_core_SmRealVector_create(&delegate->mTestState, 13, 0.0);
  sm_core_SmRealVector_create(&delegate->mFullStateVector, 13, 0.0);
  sm_core_SmRealVector_create(&delegate->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&delegate->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&delegate->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&delegate->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&delegate->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&delegate->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&delegate->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&delegate->mSvdWork, 49, 0.0);
  sm_core_SmRealVector_create(
    &delegate->mLineSearchScaledDeltaVect, 6, 0.0);
  sm_core_SmRealVector_create(
    &delegate->mLineSearchTestStateVect, 13, 0.0);
  sm_core_SmRealVector_create(&delegate->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&delegate->mActiveDofVelsVect, 6, 0.0);
  sm_core_SmRealVector_create(&delegate->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&delegate->mMotionData, 13, 0.0);
  delegate->mResetStateVector = quad_model_a487e7e2_1_resetStateVector;
  delegate->mInitializeTrackedAngleState =
    quad_model_a487e7e2_1_initializeTrackedAngleState;
  delegate->mComputeDiscreteState = quad_model_a487e7e2_1_computeDiscreteState;
  delegate->mAdjustPosition = quad_model_a487e7e2_1_adjustPosition;
  delegate->mPerturbState = quad_model_a487e7e2_1_perturbState;
  delegate->mComputeDofBlendMatrix = quad_model_a487e7e2_1_computeDofBlendMatrix;
  delegate->mProjectPartiallyTargetedPos =
    quad_model_a487e7e2_1_projectPartiallyTargetedPos;
  delegate->mPropagateMotion = quad_model_a487e7e2_1_propagateMotion;
  delegate->mComputeAssemblyError = quad_model_a487e7e2_1_computeAssemblyError;
  delegate->mComputeAssemblyJacobian =
    quad_model_a487e7e2_1_computeAssemblyJacobian;
  delegate->mComputeFullAssemblyJacobian =
    quad_model_a487e7e2_1_computeFullAssemblyJacobian;
  delegate->mIsInKinematicSingularity =
    quad_model_a487e7e2_1_isInKinematicSingularity;
  delegate->mConvertStateVector = quad_model_a487e7e2_1_convertStateVector;
  delegate->mConstructStateVector = quad_model_a487e7e2_1_constructStateVector;
  delegate->mExtractSolverStateVector =
    quad_model_a487e7e2_1_extractSolverStateVector;
  delegate->mIsPositionViolation = quad_model_a487e7e2_1_isPositionViolation;
  delegate->mIsVelocityViolation = quad_model_a487e7e2_1_isVelocityViolation;
  delegate->mProjectStateSim = quad_model_a487e7e2_1_projectStateSim;
  delegate->mMech = NULL;
}

static
  void initSimulationDelegate(SmMechanismDelegate *simDelegate)
{
  PmAllocator *alloc = pm_default_allocator();
  const SmSizePair jointToStageIdx[1] = {
    { 7, 0 }
  };

  const size_t primitiveIndices[1 + 1] = {
    0, 4
  };

  const SmSizePair stateOffsets[4] = {
    { 0, 7 }, { 1, 8 }, { 2, 9 }, { 3, 10 }
  };

  const SmSizePair dofOffsets[4] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 6 }
  };

  const size_t *remodIndices = NULL;
  const size_t *equationsPerConstraint = NULL;
  const size_t dofToVelSlot[6] = {
    7, 8, 9, 10, 11, 12
  };

  const size_t *constraintDofs = NULL;
  const size_t constraintDofOffsets[0 + 1] = {
    0
  };

  const size_t Jm = 0;
  const size_t Jn = 6;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  simDelegate->mTargetStrengthFree = 0;
  simDelegate->mTargetStrengthSuggested = 1;
  simDelegate->mTargetStrengthDesired = 2;
  simDelegate->mTargetStrengthRequired = 3;
  simDelegate->mConsistencyTol = +1.000000000000000062e-09;
  simDelegate->mDof = 6;
  simDelegate->mStateSize = 13;
  simDelegate->mNumStages = 1;
  simDelegate->mNumConstraints = 0;
  simDelegate->mNumAllConstraintEquations = 0;
  sm_core_SmSizePairVector_create(
    &simDelegate->mJointToStageIdx, simDelegate->mNumStages, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&simDelegate->mJointToStageIdx),
         jointToStageIdx, simDelegate->mNumStages * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &simDelegate->mPrimitiveIndices, simDelegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&simDelegate->mPrimitiveIndices),
         primitiveIndices, (simDelegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &simDelegate->mStateOffsets, 4, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&simDelegate->mStateOffsets),
         stateOffsets, 4 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &simDelegate->mDofOffsets, 4, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&simDelegate->mDofOffsets),
         dofOffsets, 4 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &simDelegate->mRemodIndices, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&simDelegate->mRemodIndices),
         remodIndices, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &simDelegate->mEquationsPerConstraint, simDelegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues
         (&simDelegate->mEquationsPerConstraint),
         equationsPerConstraint, simDelegate->mNumConstraints * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &simDelegate->mDofToVelSlot, simDelegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&simDelegate->mDofToVelSlot),
         dofToVelSlot, simDelegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &simDelegate->mConstraintDofs, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&simDelegate->mConstraintDofs),
         constraintDofs, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &simDelegate->mConstraintDofOffsets, simDelegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues
         (&simDelegate->mConstraintDofOffsets),
         constraintDofOffsets, (simDelegate->mNumConstraints + 1) * sizeof
         (size_t));
  sm_core_SmBoundedSet_create(&simDelegate->mPosRequired, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mPosDesired, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mPosSuggested, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mPosFree, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mPosNonRequired, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mPosSuggAndFree, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mVelRequired, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mVelDesired, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mVelSuggested, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mVelFree, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mVelNonRequired, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mVelSuggAndFree, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mConstraintFilter, 0);
  sm_core_SmBoundedSet_create(&simDelegate->mActiveConstraints, 0);
  sm_core_SmBoundedSet_create(&simDelegate->mActiveDofs, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mNewConstraints, 0);
  sm_core_SmBoundedSet_create(&simDelegate->mNewDofs, 6);
  sm_core_SmBoundedSet_create(&simDelegate->mUnsatisfiedConstraints, 0);
  sm_core_SmSizeTVector_create(&simDelegate->mActiveConstraintsVect,
    0, 0);
  sm_core_SmSizeTVector_create(&simDelegate->mActiveDofsVect, 6, 0);
  sm_core_SmSizePairVector_create(
    &simDelegate->mBaseFrameVelOffsets, 1, &zeroSizePair);
  sm_core_SmRealVector_create(&simDelegate->mInitialState, 13, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mStartState, 13, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mTestState, 13, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mFullStateVector, 13, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&simDelegate->mSvdWork, 49, 0.0);
  sm_core_SmRealVector_create(
    &simDelegate->mLineSearchScaledDeltaVect, 6, 0.0);
  sm_core_SmRealVector_create(
    &simDelegate->mLineSearchTestStateVect, 13, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mActiveDofVelsVect, 6, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&simDelegate->mMotionData, 13, 0.0);
  simDelegate->mResetStateVector = quad_model_a487e7e2_1_resetStateVector;
  simDelegate->mInitializeTrackedAngleState =
    quad_model_a487e7e2_1_initializeTrackedAngleState;
  simDelegate->mComputeDiscreteState =
    quad_model_a487e7e2_1_computeDiscreteState;
  simDelegate->mAdjustPosition = quad_model_a487e7e2_1_adjustPosition;
  simDelegate->mPerturbState = quad_model_a487e7e2_1_perturbState;
  simDelegate->mPropagateMotion = quad_model_a487e7e2_1_propagateMotion;
  simDelegate->mComputeAssemblyError =
    quad_model_a487e7e2_1_computeAssemblyError;
  simDelegate->mComputeAssemblyJacobian =
    quad_model_a487e7e2_1_computeAssemblyJacobian;
  simDelegate->mComputeFullAssemblyJacobian =
    quad_model_a487e7e2_1_computeFullAssemblyJacobian;
  simDelegate->mIsInKinematicSingularity =
    quad_model_a487e7e2_1_isInKinematicSingularity;
  simDelegate->mConvertStateVector = quad_model_a487e7e2_1_convertStateVector;
  simDelegate->mConstructStateVector =
    quad_model_a487e7e2_1_constructStateVector;
  simDelegate->mExtractSolverStateVector =
    quad_model_a487e7e2_1_extractSolverStateVector;
  simDelegate->mIsPositionViolation = quad_model_a487e7e2_1_isPositionViolation;
  simDelegate->mIsVelocityViolation = quad_model_a487e7e2_1_isVelocityViolation;
  simDelegate->mProjectStateSim = quad_model_a487e7e2_1_projectStateSim;
  simDelegate->mMech = NULL;
}

static
  void initAssemblyStructures(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T *motionInputOffsets = NULL;
  int_T status = 0;
  initMechanismDelegate(&smData->mMechanismDelegate);
  initSimulationDelegate(&smData->mSimulationDelegate);
  status = pm_create_int_vector_fields(
    &smData->mMotionInputOffsets, smData->mNumInputMotionPrimitives, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mMotionInputOffsets.mX, motionInputOffsets,
         0 * sizeof(int32_T));
}

static
  void initComputationFcnPtrs(NeDaePrivateData *smData)
{
  smData->mDerivativeFcn = dae_cg_deriv_method;
  smData->mOutputFcn = dae_cg_output_method;
  smData->mProjectionFcn = dae_cg_project_solve;
  smData->mProjectionMaybeFcn = dae_cg_projectMaybe_solve;
  smData->mCheckFcn =
    (smData->mStateVectorSize == 0) ? dae_cg_check_solve : NULL;
  smData->mAssemblyFcn = dae_cg_assemble_solve;
  smData->mSetupLoggerFcn = sm_ssci_setupLoggerFcn_codeGen;
  smData->mLogFcn = sm_ssci_logFcn_codeGen;
  smData->mResidualsFcn = NULL;
  smData->mLinearizeFcn = NULL;
  smData->mGenerateFcn = NULL;
}

static
  void initLiveLinkToSm(NeDaePrivateData *smData)
{
  smData->mLiveSmLink = NULL;
  smData->mLiveSmLink_destroy = NULL;
  smData->mLiveSmLink_copy = NULL;
}

void quad_model_a487e7e2_1_NeDaePrivateData_create(NeDaePrivateData *smData)
{
  initBasicAttributes (smData);
  initStateVector (smData);
  initVariables (smData);
  initIoInfo (smData);
  initInputDerivs (smData);
  initDirectFeedthrough (smData);
  initOutputDerivProc (smData);
  initAssemblyStructures (smData);
  initComputationFcnPtrs (smData);
  initLiveLinkToSm (smData);
}
