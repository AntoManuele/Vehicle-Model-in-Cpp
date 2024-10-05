#ifndef _RACINGCAR_H
#define _RACINGCAR_H

#include <iostream>
#include <math.h>
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



class RacingCar {

protected:
    float                   m_fTimeStepS;
    RacingCar_PARAM_t       CarParam;
    RacingCar_STATE_t       CarState;
    Dynamics_Var_t          DynVar;

public:
    /* Class constructor */
    RacingCar(float p_fTimeStepS) 
    {
        this->m_fTimeStepS = p_fTimeStepS;
    }


    /* Function to iniziale the object and set all the parameters of the car */
    virtual bool SetParameters() = 0;


    /* SetInitialAttitude() */
    void SetInitialAttitude(float p_fInitialPosXM, float p_fInitialPosYM, float p_fInitialYawRad)
    {
        CarState.m_fPositionXM  = p_fInitialPosXM;
        CarState.m_fPositionYM  = p_fInitialPosYM;
        CarState.m_fYawRad      = p_fInitialYawRad;
        
        return;
    }


    /* SetInitialVelocity() */
    bool SetInitialVelocity(float p_fInitialLongVelMs)
    {
        if (p_fInitialLongVelMs <= 0)
        {
            return false;
        }
        else if (p_fInitialLongVelMs < 10)
        {
            CarState.m_fLongitudinalVelocityMs = 10; // to avoid singularities
        }
        else
        {
            CarState.m_fLongitudinalVelocityMs = p_fInitialLongVelMs;
        }
    }


    float FromThrottleToEngineForce(float p_fThrottle)
    {
        float   l_fEngineTorqueNm;
        float   l_fWheelForceN;
        float   l_EngineForce;

        l_fEngineTorqueNm   = this->CarParam.m_fMaxEngineTorqueNm * p_fThrottle;

        l_fWheelForceN      = (l_fEngineTorqueNm * this->CarParam.m_fGearRatio) / this->CarParam.m_fWheelRadiusM;

        l_EngineForce       = l_fWheelForceN * this->CarParam.m_fTransmissionEfficiency;

        return l_EngineForce;
    }


    /* Kynematics */
    void Kynematics()
    {
        float   l_fPosXM, l_fPosYM, l_fYawRad;
        float   x0, y0, psi0;
        float   u, v, r, dt;

        x0      = this->CarState.m_fPositionXM;
        y0      = this->CarState.m_fPositionYM;
        psi0    = this->CarState.m_fYawRad;

        u       = this->CarState.m_fLongitudinalVelocityMs;
        v       = this->CarState.m_fLateralVelocityMs;
        r       = this->CarState.m_fYawRateRadS;

        dt      = this->m_fTimeStepS;


        l_fPosXM    = x0 + (u*cos(psi0) - v*sin(psi0)) * dt;
        l_fPosYM    = y0 + (u*sin(psi0) + v*cos(psi0)) * dt;
        l_fYawRad   = psi0 + r * dt;


        this->CarState.m_fPositionXM     = l_fPosXM;
        this->CarState.m_fPositionYM     = l_fPosYM;
        this->CarState.m_fYawRad         = l_fYawRad;

    }


    
    virtual void Dynamics(RacingCar_ThrottleDriven_INP_t DynInp) = 0;



    void SetModelOutput(RacingCar_OUT_t *CarOut)
    {
        /* Position XY and yaw */
        CarOut->m_fPositionXM       = this->CarState.m_fPositionXM;
        CarOut->m_fPositionYM       = this->CarState.m_fPositionYM;
        CarOut->m_fYawRad           = this->CarState.m_fYawRad;

        /* Velocity components and yaw rate */
        CarOut->m_fLongitudinalVelocityMs   = this->CarState.m_fLongitudinalVelocityMs;
        CarOut->m_fLateralVelocityMs        = this->CarState.m_fLateralVelocityMs;
        CarOut->m_fYawRateRadS              = this->CarState.m_fYawRateRadS;

        /* Accelerations ax, ay, rdot */
        CarOut->m_fLongitudinalAccelerationMs2 = // udot - vr
            this->DynVar.m_fLongitudinalVelocityDotMs2 - this->DynVar.m_fLateralVelocityMs * this->DynVar.m_fYawRateRadS;

        CarOut->m_fLateralAccelerationMs2 = // vdot + ur
            this->DynVar.m_fLateralVelocityDotMs2 + this->DynVar.m_fLongitudinalVelocityMs * this->DynVar.m_fYawRateRadS;

        CarOut->m_fYawAccelerationRadS2 =
            this->DynVar.m_fYawAccelerationRadS2;

    }



    virtual void Run(RacingCar_ThrottleDriven_INP_t Inp, RacingCar_OUT_t *CarOutput) = 0;


};




#endif