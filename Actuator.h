/*************************************************************************************************************************************************************/
//  Created by: Ryan Spillman
//
//  Last updated 4/9/2015
//
//  Contains functions for controlling L298n H-bridge which controls the linear actuator
/*************************************************************************************************************************************************************/

#define Actuator_h

//L298n connections
//mbed classes (mbed folder)
DigitalOut pinI1(D7);
DigitalOut pinI2(PTC12);    //D8
PwmOut ENA(D6);

/*************************************************************************************************************************************************************/
//                                                      L298n (H-Bridge) Functions
/*************************************************************************************************************************************************************/

void turnStop(float valueOne, float valueTwo)
{
    pinI1 = 0;
    pinI2 = 0;
    ENA =  valueOne;
}

void turnLeft(float valueOne, float valueTwo)
{
    pinI1 = 0;
    pinI2 = 1;
    ENA =  valueOne;
}

void turnRight(float valueOne, float valueTwo)
{
    pinI1 = 1;
    pinI2 = 0;
    ENA =  valueOne;
}

//calculate an equal distance from acuator center point that the acutator can physically move to without hitting limit switch
float calcEnds(float center, float max, float min)
{
    float upperRange = max - center;
    float lowerRange = center - min;
    if(upperRange < lowerRange)
    {
        return upperRange;
    }
    else
    {
        return lowerRange;
    }
}