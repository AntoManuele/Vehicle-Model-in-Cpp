#ifndef _SINGLETRACK_H
#define _SINGLETRACK_H

#include <iostream>
#include <math.h>


class SingleTrack : public RacingCar {

public:
    /* Class constructor */
    SingleTrack(float p_fTimeStepS) : RacingCar(p_fTimeStepS) {}

    /* Function to iniziale the object and set all the parameters of the car */
    bool SetParameters(RacingCar_PARAM_t *CarInit)
    {
        if (CarInit->m_fMassKg <= 0 || CarInit->m_fWheelbaseM <= 0 ||
            CarInit->m_fInertiaMomentKgM2 <= 0 || CarInit->m_fWheelbaseM ||
            CarInit->m_fFrontWheelbaseM <= 0 || CarInit->m_fRearWheelbaseM <= 0 ||
            CarInit->m_fFrontTrackWidthM <= 0 || CarInit->m_fRearTrackWidthM <= 0)
        {
            return false;
        }
        else
        {
            this->CarParam.m_fMassKg                      = CarInit->m_fMassKg;
            this->CarParam.m_fInertiaMomentKgM2           = CarInit->m_fInertiaMomentKgM2;

            this->CarParam.m_fWheelbaseM                  = CarInit->m_fWheelbaseM;
            this->CarParam.m_fFrontWheelbaseM             = CarInit->m_fFrontWheelbaseM;
            this->CarParam.m_fRearWheelbaseM              = CarInit->m_fRearWheelbaseM;

            this->CarParam.m_fFrontTrackWidthM            = CarInit->m_fFrontTrackWidthM;
            this->CarParam.m_fRearTrackWidthM             = CarInit->m_fRearTrackWidthM;

            this->CarParam.m_fSteeringRatio               = CarInit->m_fSteeringRatio;
            this->CarParam.m_fFrontCorneringStiffnessN    = CarInit->m_fFrontCorneringStiffnessN;
            this->CarParam.m_fRearCorneringStiffnessN     = CarInit->m_fRearCorneringStiffnessN;
            this->CarParam.m_fBrakeBalance                = CarInit->m_fBrakeBalance;

            this->CarParam.m_fTimeStepS                   = CarInit->m_fTimeStepS;

        }

        return true;

    }


    void Dynamics(RacingCar_ThrottleDriven_INP_t DynInp)
    {
        float   Fx1, Fx2;            // overall longitudinal force (front and rear)
        float   alpha1, alpha2;      // slip angles
        float   u, v, r;
        float   uDot, vDot, rDot;
        float   delta, throttle;
        float   m, J, a, b, dt;
        float   C1, C2;             // cornering stiffness

        u           = this->CarState.m_fLongitudinalVelocityMs;
        v           = this->CarState.m_fLateralVelocityMs;
        r           = this->CarState.m_fYawRateRadS;

        delta       = DynInp.m_fSteeringAngleRad;
        throttle    = DynInp.m_fThrottlePedal;

        m           = this->CarParam.m_fMassKg;
        J           = this->CarParam.m_fInertiaMomentKgM2;
        a           = this->CarParam.m_fFrontWheelbaseM;
        b           = this->CarParam.m_fRearWheelbaseM;

        C1          = this->CarParam.m_fFrontCorneringStiffnessN;
        C2          = this->CarParam.m_fRearCorneringStiffnessN;

        dt          = this->CarParam.m_fTimeStepS;

        Fx1     = this->CarParam.m_fBrakeBalance * DynInp.m_fBrakePedal;
        Fx2     = this->FromThrottleToEngineForce(throttle) + (1 - this->CarParam.m_fBrakeBalance) * DynInp.m_fBrakePedal;

        /* Slip Angles */
        alpha1  = delta - ((v + r*a) / u);
        alpha2  = - (v - r*b) / u;


        /* Velocity derivatives, Single Track Model */
        uDot    = (1/m) * (Fx1*cos(delta) + Fx2 + C1*alpha1*sin(delta)) + v*r;
        vDot    = (1/m) * (C1*alpha1*cos(delta) + C2*alpha2 - Fx1*sin(delta)) - u*r;
        rDot    = (1/J) * (C1*alpha1*cos(delta)*a - C2*alpha2*b + Fx1*sin(delta)*a);


        /* Update output */
        this->DynVar.m_fLongitudinalVelocityMs   = u + uDot * dt;
        this->DynVar.m_fLateralVelocityMs        = v + vDot * dt;
        this->DynVar.m_fYawRateRadS              = r + rDot * dt;

        this->DynVar.m_fLongitudinalVelocityDotMs2   = uDot;
        this->DynVar.m_fLateralVelocityDotMs2        = vDot;
        this->DynVar.m_fYawAccelerationRadS2         = rDot;

        this->CarState.m_fLongitudinalVelocityMs = this->DynVar.m_fLongitudinalVelocityMs;
        this->CarState.m_fLateralVelocityMs      = this->DynVar.m_fLateralVelocityMs;
        this->CarState.m_fYawRateRadS            = this->DynVar.m_fYawRateRadS;

        return;
    }



    void Run(RacingCar_ThrottleDriven_INP_t Inp, RacingCar_OUT_t *CarOutput)
    {
        Dynamics(Inp);
        RacingCar::Kynematics();

        RacingCar::SetModelOutput(CarOutput);

    }


};



#endif