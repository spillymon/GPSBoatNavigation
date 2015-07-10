/**************************************************************************************************************************************************************
//  This is a modified version of mbed /users/aberk/code/PID/ for LSM303DLHC
//
//  Changes made by Ryan Spillman:
//  Fixed a mathematical issue (need to better document said fix)
//
//  Need to better document scaling and other mathematical issues
**************************************************************************************************************************************************************/
#include "PID.h"

PID::PID(float Kc, float tauI, float tauD, float interval) 
{

    usingFeedForward = false;
    inAuto           = false;

    //Default the limits to the full range of I/O: 3.3V
    //Make sure to set these to more appropriate limits for
    //your application.
    setInputLimits(0.0, 3.3);
    setOutputLimits(0.0, 3.3);

    tSample_ = interval;

    setTunings(Kc, tauI, tauD);

    setPoint_             = 0.0;
    processVariable_      = 0.0;
    prevProcessVariable_  = 0.0;
    controllerOutput_     = 0.0;
    prevControllerOutput_ = 0.0;

    accError_ = 0.0;
    bias_     = 0.0;
    
    realOutput_ = 0.0;

}

void PID::setInputLimits(float inMin, float inMax) 
{
    inMin_  = inMin;
    inMax_  = inMax;
    inSpan_ = inMax - inMin;

}

void PID::setOutputLimits(float outMin, float outMax) 
{

    outMin_  = outMin;
    outMax_  = outMax;
    outSpan_ = outMax - outMin;

}

void PID::setTunings(float Kc, float tauI, float tauD) 
{
    //Store raw values to hand back to user on request.
    pParam_ = Kc;
    iParam_ = tauI;
    dParam_ = tauD;

    float tempTauR;

    if (tauI == 0.0f) {
        tempTauR = 0.0f;
    } else {
        tempTauR = (1.0f / tauI) * tSample_;
    }

    //For "bumpless transfer" we need to rescale the accumulated error.
    if (inAuto) {
        if (tempTauR == 0.0f) {
            accError_ = 0.0f;
        } else {
            accError_ *= (Kc_ * tauR_) / (Kc * tempTauR);
        }
    }

    Kc_   = Kc;
    tauR_ = tempTauR;
    tauD_ = tauD / tSample_;

}

void PID::reset(void) 
{
    prevControllerOutput_ = 0.0f;

    //Clear any error in the integral.
    accError_ = 0.0f;

}

void PID::setMode(int mode) {

    //We were in manual, and we just got set to auto.
    //Reset the controller internals.
    if (mode != 0 && !inAuto) {
        reset();
    }

    inAuto = (mode != 0);

}

void PID::setInterval(float interval) {

    if (interval > 0) {
        //Convert the time-based tunings to reflect this change.
        tauR_     *= (interval / tSample_);
        accError_ *= (tSample_ / interval);
        tauD_     *= (interval / tSample_);
        tSample_   = interval;
    }

}

void PID::setSetPoint(float sp) {

    setPoint_ = sp;

}

void PID::setProcessValue(float pv) {

    processVariable_ = pv;

}

void PID::setBias(float bias){

    bias_ = bias;
    usingFeedForward = 1;

}

float PID::compute() 
{
    float error = setPoint_ - processVariable_;

    //Check and see if the output is pegged at a limit and only
    //integrate if it is not. This is to prevent reset-windup.
    if (!(prevControllerOutput_ >= outMax_ && error > 0) && !(prevControllerOutput_ <= outMin_ && error < 0)) {
        accError_ += error;
    }

    //Compute the current slope of the input signal.
    float dMeas = (processVariable_ - prevProcessVariable_) / tSample_;

    //Perform the PID calculation.
    controllerOutput_ = Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));
    
    //scale PID output based on user provided input and output restraints
    controllerOutput_ = ((outMax_ - outMin_) * ((Kc_ * controllerOutput_) - inMin_)) / (inMax_ - inMin_) + outMin_;
    
    //Make sure the computed output is within output constraints.
    if (controllerOutput_ < outMin_) {
        controllerOutput_ = outMin_;
    } else if (controllerOutput_ > outMax_) {
        controllerOutput_ = outMax_;
    }
    
    //Remember this output for the windup check next time.
    prevControllerOutput_ = controllerOutput_;
    //Remember the input for the derivative calculation next time.
    prevProcessVariable_  = processVariable_;
    
    //Scale the output from percent span back out to a real world number.
    return controllerOutput_;
}

float PID::getInMin() {

    return inMin_;

}

float PID::getInMax() {

    return inMax_;

}

float PID::getOutMin() {

    return outMin_;

}

float PID::getOutMax() {

    return outMax_;

}

float PID::getInterval() {

    return tSample_;

}

float PID::getPParam() {

    return pParam_;

}

float PID::getIParam() {

    return iParam_;

}

float PID::getDParam() {

    return dParam_;

}
