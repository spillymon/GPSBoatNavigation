/*************************************************************************************************************************************************************/
//  Created by: Ryan Spillman
//
//  Last updated 4/29/2015
//
//  Contains several functions such as calculations for roll, pitch, and yaw (tilt compensated compass)
//  Also contains various filtering and calibration functions
//
//  Requires L3GD20 gyro and LSM303DLHC magnetometer and accelerometer
/*************************************************************************************************************************************************************/

#include "mbed.h"
#include "math.h"
#include "L3GD20.h"
#include "LSM303DLHC.h"

#define IMUDataAndFilters_h

L3GD20 gyro(PTE25, PTE24);                              //L3GD20/L3GD20.cpp
LSM303DLHC compass(PTE25, PTE24);                       //LSM303DLHC/LSM303DLHC.cpp

#define ALPHA_H             0.2f                        //Heading alpha (variable for low pass filter)
#define ALPHA_A             0.1f                        //Heading accelerometer (variable for low pass filter)
#define EARTHRADIUS     6378100.000000000000000000f     //Radius of the earth in meters (DO NOT MODIFY)
#define MAGN_PERIOD     (1 / 15.0f)                     //magnetometer polling period (15 Hz) must manually update hardware ODR registers if chaging this value
#define ACCEL_PERIOD    (1 / 50.0f)                     //accelerometer polling period (50 Hz) must manually update hardware ODR registers if chaging this value
#define GYRO_SAMPLE_PERIOD  0.01f                       //gyro sample period in seconds
#define M_PI                3.1415926535897932384626433832795f
#define TWO_PI              6.283185307179586476925286766559f
#define RADTODEGREE         57.295779513082320876798154814105f
#define DEGREETORAD         0.01745329251994329576923690768489f
#define GRAVITY             1.0f
#define CALIBRATION_COUNT   64
#define AVG_COUNT           10

// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN         -1.000000000f
#define ACCEL_X_MAX         1.023437500f
#define ACCEL_Y_MIN         -1.015625000f
#define ACCEL_Y_MAX         1.023437500f
#define ACCEL_Z_MIN         -1.023437500f 
#define ACCEL_Z_MAX         0.960937500f

// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN          (-0.370909f)
#define MAGN_X_MAX          (0.569091f)
#define MAGN_Y_MIN          (-0.516364f)
#define MAGN_Y_MAX          (0.392727f)
#define MAGN_Z_MIN          (-0.618367f)
#define MAGN_Z_MAX          (0.408163f)

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
//not sure what the "normal" value is, thus not using scale for magnetometer
//#define MAGN_X_SCALE (??? / (MAGN_X_MAX - MAGN_X_OFFSET))
//#define MAGN_Y_SCALE (??? / (MAGN_Y_MAX - MAGN_Y_OFFSET))
//#define MAGN_Z_SCALE (??? / (MAGN_Z_MAX - MAGN_Z_OFFSET))


float magnetom[3] = {0,0,0};
float accel[3] = {0,0,0};
float degGyro[3] = {0,0,0};
float prevDegGyro[3]= {0,0,0};
float gyroOffset[3] = {0,0,0};
float roll = 0, pitch = 0, yaw = 0;
float totalAccel = 0;
float normalizedGravity;

float lowPassAccel[3] = {0.0000001f,0.0000001f,0.0000001f};

float prevYaw = 0;

/********************************************************************************************************************************/
//                                          Low Pass
/********************************************************************************************************************************/

float lowPass(float input, float output, int select) 
{
    //Magnetometer Heading alpha
    if(select == 0)
    {
        if (output == 0.0000001f) return input;
        
        output = output + ALPHA_H * (input - output);
        
        return output;
    }
    //Accelerometer alpha
    else if(select == 1)
    {
        if (output == 0.0000001f) return input;
        
        output = output + ALPHA_A * (input - output);
        
        return output;
    }
    //GPS heading alpha
    else
    {
        if (output == 0.0000001f) return input;
        
        output = output + ALPHA_A * (input - output);
        
        return output;
    }
}

/********************************************************************************************************************************/
//                                          getMagn
/********************************************************************************************************************************/

void getMagn()
{
    //LSM303DLHC/LSM303DLHC.cpp
    compass.ReadMagnOnly(&magnetom[0], &magnetom[1], &magnetom[2]);

    //Compensate magnetometer error
    //magnetom[0] -= MAGN_X_OFFSET;
    //magnetom[1] -= MAGN_Y_OFFSET;
    //magnetom[2] -= MAGN_Z_OFFSET;
}
/********************************************************************************************************************************/
//                                          getAccel
/********************************************************************************************************************************/

void getAccel()
{
    //LSM303DLHC/LSM303DLHC.cpp
    compass.ReadAccOnly(&accel[0], &accel[1], &accel[2]);
    
    //Compensate accelerometer error
    //TODO: verify compensation calculations
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    
    for(int i = 0; i <= 3; i++)
    {
        lowPassAccel[i] = lowPass(accel[i], lowPassAccel[i], 1);
    }    
}

/********************************************************************************************************************************/
//                                                      Integrate Yaw With Gyro Data
/********************************************************************************************************************************/
//not used
void gyroIntegral()
{
    float yawDiff = yaw - prevYaw;
    float absYawDiff = sqrt(yawDiff * yawDiff);
    //vC[x] = vP[x] + ((aP[x] + ((inertialAccel[x] - aP[x]) / 2)) * deltaT);
    float gyroInt = prevYaw + ((prevDegGyro[2] + ((degGyro[2] - prevDegGyro[2]) / 2)) * MAGN_PERIOD);
    float absGyroInt = sqrt(gyroInt * gyroInt);
    prevDegGyro[2] = degGyro[2];
    if(absYawDiff > absGyroInt)
    {
        yaw = prevYaw + gyroInt;
    }
}

/********************************************************************************************************************************/
//                                                      Compass Heading
/********************************************************************************************************************************/
//see http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf for more info on these formulas
void Compass_Heading()
{
    float mag_x;
    float mag_y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
    
    //saves a few processor cycles by calculating and storing values
    cos_roll = cos(roll);
    sin_roll = sin(roll);
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);
    
    // Tilt compensated magnetic field X
    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
    
    // Tilt compensated magnetic field Y
    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;

    // Magnetic Heading
    yaw = atan2(-mag_x, mag_y);
    
    //make negative angles positive
    if(yaw < 0) yaw = yaw + TWO_PI;
    //yaw = yaw + M_PI;
    yaw = yaw * RADTODEGREE;
}

/********************************************************************************************************************************/
//                                                      getAttitude
/********************************************************************************************************************************/
//see http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf for more info on these formulas
void getAttitude()
{
    float temp1[3];
    float temp2[3];
    float xAxis[3] = {1.0f, 0.0f, 0.0f};
    
    // GET PITCH
    // Using y-z-plane-component/x-component of gravity vector    
    pitch = -atan2(accel[0], sqrt((accel[1] * accel[1]) + (accel[2] * accel[2])));
    
    //printf("P = %f", pitch);
    
    // GET ROLL
    // Compensate pitch of gravity vector 
    temp1[0] = (accel[1] * xAxis[2]) - (accel[2] * xAxis[1]);
    temp1[1] = (accel[2] * xAxis[0]) - (accel[0] * xAxis[2]);
    temp1[2] = (accel[0] * xAxis[1]) - (accel[1] * xAxis[0]);
    
    temp2[0] = (xAxis[1] * temp1[2]) - (xAxis[2] * temp1[1]);
    temp2[1] = (xAxis[2] * temp1[0]) - (xAxis[0] * temp1[2]);
    temp2[2] = (xAxis[0] * temp1[1]) - (xAxis[1] * temp1[0]);
    
    // Since we compensated for pitch, x-z-plane-component equals z-component:
    roll = atan2(temp2[1], temp2[2]);
}

/********************************************************************************************************************************/
//                                                      Filtered_Attitude()
/********************************************************************************************************************************/

void Filtered_Attitude()
{
    float temp1[3];
    float temp2[3];
    float xAxis[3] = {1.0f, 0.0f, 0.0f};
    
    // GET PITCH
    // Using y-z-plane-component/x-component of gravity vector    
    pitch = -atan2(lowPassAccel[0], sqrt((lowPassAccel[1] * lowPassAccel[1]) + (lowPassAccel[2] * lowPassAccel[2])));
    
    //printf("P = %f", pitch);
    
    // GET ROLL
    // Compensate pitch of gravity vector 
    temp1[0] = (lowPassAccel[1] * xAxis[2]) - (lowPassAccel[2] * xAxis[1]);
    temp1[1] = (lowPassAccel[2] * xAxis[0]) - (lowPassAccel[0] * xAxis[2]);
    temp1[2] = (lowPassAccel[0] * xAxis[1]) - (lowPassAccel[1] * xAxis[0]);
    
    temp2[0] = (xAxis[1] * temp1[2]) - (xAxis[2] * temp1[1]);
    temp2[1] = (xAxis[2] * temp1[0]) - (xAxis[0] * temp1[2]);
    temp2[2] = (xAxis[0] * temp1[1]) - (xAxis[1] * temp1[0]);
    
    // Since we compensated for pitch, x-z-plane-component equals z-component:
    roll = atan2(temp2[1], temp2[2]);
}

/********************************************************************************************************************************/
//                                                      updateAngles
/********************************************************************************************************************************/

void updateAngles()
{
    Filtered_Attitude();
    Compass_Heading();
}

/********************************************************************************************************************************/
//                                                      normalizeGravity
/********************************************************************************************************************************/
//not used
void normalizeGravity()
{
    float accumulator = 0;
    int sampleCount  = 0;

    
    //We'll take a certain number of samples and then average them to calculate the average
    while (sampleCount < AVG_COUNT) 
    {
        getAccel();
        //add current sample to previous samples
        accumulator += sqrt((accel[0] * accel[0]) + (accel[1] * accel[1]) + (accel[2] * accel[2]));
        sampleCount++;
        //Make sure the IMU has had enough time to take a new sample.
        wait(0.06);
    }
    

    //divide by number of samples to get average offset
    normalizedGravity = accumulator / AVG_COUNT;

}
/********************************************************************************************************************************/
//                                                      gyroAvg
/********************************************************************************************************************************/
//not used
void gyroAvg()
{
    float accumulator[3] = {0,0,0};
    int sampleCount  = 0;

    
    //We'll take a certain number of samples and then average them to calculate the offset
    while (sampleCount < AVG_COUNT) 
    {
        //get gyro data
        gyro.read(&degGyro[0], &degGyro[1], &degGyro[2]);
        //add current sample to previous samples
        accumulator[2] += degGyro[2];
        sampleCount++;
        //Make sure the gyro has had enough time to take a new sample.
        wait(GYRO_SAMPLE_PERIOD);
    }
    

    //divide by number of samples to get average offset
    degGyro[2] = accumulator[2] / AVG_COUNT;

}

/********************************************************************************************************************************/
//                                                      gyroCal
/********************************************************************************************************************************/
//not used
void gyroCal()
{
    float accumulator[3] = {0,0,0};
    int sampleCount  = 0;

    
    //We'll take a certain number of samples and then average them to calculate the offset
    while (sampleCount < CALIBRATION_COUNT) 
    {
        float gyroData[3];
        //Make sure the gyro has had enough time to take a new sample.
        wait(GYRO_SAMPLE_PERIOD);
        //get gyro data
        gyro.read(&gyroData[0],&gyroData[1],&gyroData[2]);
        for(int i = 0; i < 3; i++)
        {  
            //add current sample to previous samples
            accumulator[i] += gyroData[i];
        }
        sampleCount++;
    }
    
    for(int i = 0; i < 3; i++)
    {
        //divide by number of samples to get average offset
        gyroOffset[i] = accumulator[i] / CALIBRATION_COUNT;
    }
}

/********************************************************************************************************************************/
//                                                      Only Get Gyro Data
/********************************************************************************************************************************/
//not used
void gyroOnly()
{
    gyro.read(&degGyro[0], &degGyro[1], &degGyro[2]);
    //compensate for gyro offset
    for(int i=0;i<=3;i++)
    {
        degGyro[i] -= gyroOffset[i];
    }
}