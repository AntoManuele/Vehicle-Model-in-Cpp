#ifndef _DOUBLETRACK_H
#define _DOUBLETRACK_H

#include <iostream>
#include <math.h>


typedef struct DoubleTrack_PARAM_struct 
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

    /* Parameters of the DoubleTrack */
    float   m_fToeRad;
    float   m_fCamberRad;

    /* Magic Formula */
    float   m_fMagicFormulaD;
    float   m_fMagicFormulaC;
    float   m_fMagicFormulaB;
    float   m_fMagicFormulaE;

    /* Aeroodynamic Parameters */
    float   m_fAirDensityKgm3;
    float   m_fFrontSurfaceM2;
    float   m_fDragCoeffCx;
    float   m_fDownForceCoeffCz1;
    float   m_fDownForceCoeffCz2;

    float   m_fTimeStepS;

} DoubleTrack_PARAM_t;


struct DoubleTrack_STEER_struct
{
    float   m_fDragForceXaN;
    float   m_fDownForceZ1N;
    float   m_fDownForceZ2N;  
} DoubleTrack_STEER_t;


struct DoubleTrack_AERO_struct
{
    float   m_fDragForceXaN;
    float   m_fDownForceZ1N;
    float   m_fDownForceZ2N;  
} DoubleTrack_AERO_t;


struct DoubleTrack_LOAD_struct
{
    float   m_fLongLoadTransferN;
    float   m_fFrontLateralLoadTransferN;
    float   m_fRearLateralLoadTransferN;  
} DoubleTrack_LOAD_t;


struct DoubleTrack_VERTFORCES_struct
{
    float   m_fFrontLeftVertForceZ11;
    float   m_fFrontRightVertForceZ12;
    float   m_fRearLeftVertForceZ21;  
    float   m_fRearRightVertForceZ22;
} DoubleTrack_VERTFORCES_t;



/* ------------- Definition of class DoubleTrack ------------- */
class DoubleTrack : public RacingCar {

private:
    DoubleTrack_PARAM_t     CarParam;

public:
    /* Class constructor */
    DoubleTrack(float p_fTimeStepS) : RacingCar(p_fTimeStepS) {}


    /* Function to iniziale the object and set all the parameters of the car */
    bool SetParameters(DoubleTrack_PARAM_t *CarInit)
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

            this->CarParam.m_fToeRad                      = CarInit->m_fToeRad;
            this->CarParam.m_fCamberRad                   = CarInit->m_fCamberRad;

            this->CarParam.m_fMagicFormulaD               = CarInit->m_fMagicFormulaD;
            this->CarParam.m_fMagicFormulaC               = CarInit->m_fMagicFormulaC;
            this->CarParam.m_fMagicFormulaB               = CarInit->m_fMagicFormulaB;
            this->CarParam.m_fMagicFormulaE               = CarInit->m_fMagicFormulaE;

            this->CarParam.m_fAirDensityKgm3              = CarInit->m_fAirDensityKgm3;
            this->CarParam.m_fFrontSurfaceM2              = CarInit->m_fFrontSurfaceM2;
            this->CarParam.m_fDragCoeffCx                 = CarInit->m_fDragCoeffCx;
            this->CarParam.m_fDownForceCoeffCz1           = CarInit->m_fDownForceCoeffCz1;
            this->CarParam.m_fDownForceCoeffCz2           = CarInit->m_fDownForceCoeffCz2;

        }

        return true;
    }

/*
    void GetSteerAngles(float p_fDeltaRad, DoubleTrack_STEER_t *SteerAngles)
    {

    }


    void GetAerodynamics(float p_fLongVelocity, DoubleTrack_AERO_t *AeroForces)
    {

    }


    float MagicFormula(float p_fSlip, float p_fVerticalForceN)
    {

        
    }

    float GetLoadTransfers()
    {
    }


    void GetVerticalForces(float p_fLongVelocity, DoubleTrack_LOAD_t LoadTransfers, DoubleTrack_VERTFORCES_t *VertForces)
    {

    }

*/


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