#ifndef _RACINGCAR_H
#define _RACINGCAR_H

#include <stdbool.h>


typedef struct RacingCar_PARAM_struct 
{
    float   m_fMassKg;
    float   m_fInertiaMomentKgM2;
    float   m_fWheelbaseM;
    float   m_fFrontWheelbaseM;
    float   m_fRearWheelbaseM;
    float   m_fFrontTrackWidthM;
    float   m_fRearTrackWidthM;
    float   m_fSteeringRatio;
    float   m_fFrontCorneringStiffnessN;
    float   m_fRearCorneringStiffnessN;
    float   m_fWheelRadiusM;
    float   m_fBrakeBalance;

    float   m_fMaxEngineTorqueNm;
    float   m_fGearRatio;
    float   m_fTransmissionEfficiency;

    float   m_fTimeStepS;

} RacingCar_PARAM_t;


typedef struct RacingCar_STATE_struct 
{
    float   m_fPositionXM;
    float   m_fPositionYM;
    float   m_fYawRad;

    float   m_fLongitudinalVelocityMs;
    float   m_fLateralVelocityMs;
    float   m_fYawRateRadS;

} RacingCar_STATE_t;



/* Throttle Driven */
typedef struct RacingCar_ThrottleDriven_INP_struct
{
    float   m_fThrottlePedal;
    float   m_fBrakePedal;
    float   m_fSteeringAngleRad;

} RacingCar_ThrottleDriven_INP_t;


typedef struct Dynamics_INP_struct
{
    float   m_fThrottlePedal;
    float   m_fBrakePedal;
    float   m_fSteeringAngleRad;
    
} Dynamics_INP_t;


typedef struct Dynamics_OUT_struct
{
    float   m_fLongitudinalVelocityMs;
    float   m_fLateralVelocityMs;
    float   m_fYawRateRadS;

    float   m_fLongitudinalVelocityDotMs2;
    float   m_fLateralVelocityDotMs2;
    float   m_fYawAccelerationRadS2;
} Dynamics_Var_t;

  
/* Car Output */
typedef struct RacingCar_OUT_struct
{
    float   m_fPositionXM;
    float   m_fPositionYM;
    float   m_fYawRad;

    float   m_fLongitudinalVelocityMs;
    float   m_fLateralVelocityMs;
    float   m_fYawRateRadS;

    float   m_fLongitudinalAccelerationMs2;
    float   m_fLateralAccelerationMs2;
    float   m_fYawAccelerationRadS2;

} RacingCar_OUT_t;


#endif