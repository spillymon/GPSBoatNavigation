/************************************************************************************************************************************************************************************************/
//  Created by: Ryan Spillman
//
//  Last updated 4/29/2015
//
//  This is the software for my teams autonomous boat that is our graduation/final project at Isothermal Community College 
//
//  The user can drive the vehicle by sending chars over the xBee's serial connection
//  GPS waypoints are stored on an external micro-SD card
//  The user can record new waypoints to the SD card by driving to a location and entering record mode
//  The user can also manually adjust the waypoints with a text editor
//
//  A PID loop is used to control the heading of the vehicle
//
//  The project uses a FRDM-K64f (Freescale microcontroller), a LSM303DLHC (magnetometer and accelerometer) to create a tilt compensated comapass, 
//  MTK3339 GPS module, two xBee Pro S1, three TE KRPA-11 relays (relay logic H-bridge for trolling motor), and a L298n (H-Bridge for linear actuator)
//
/**************************************************!!!!!!!WARNING!!!!!!**************************************************************************************************************************/
//  DO NOT update mbed or SDFileSystem folder
//  Doing so will/may break this program
/**************************************************!!!!!!!WARNING!!!!!!**************************************************************************************************************************/
//
/***************************************************How To***************************************************************************************************************************************/
//
//  Requires a serial to usb adapter to connect an X-Bee to a PC
//  Set both X-Bees up for 115200 baud
//  Use TeraTerm (or other serial program) to read and send data over X-Bees
//  Set TeraTerm new line character from CR (carrage return) to LF (line feed)
//  Program starts by prompting user to press any key
//  Once user presses a key, the program waits for a DGPS fix (can be set by changing "FIX")
//  Once the program sees a DGPS fix, manual mode is enabled
//  User can drive the vehicle in manual mode to any position
//  User can record current position to a selected goal position in record mode
//  In autonomous mode, the vehicle uses GPS data and compass data to navigate to each goal position
//
//  Controls in manual mode:
//      directional:
//          w = forward
//          s = backward
//          a = left
//          d = right
//      mode:
//          r = change to record mode
//          z = change to autonomous mode
//
//  Controls in autonomous mode:
//  mode:
//      y = change to manual mode
//  adjustments:
//      d = increase angle
//      a = decrease angle
//      r = enter new waypoint number
//      + = increase (depends on adjust mode)
//      - = decrease (depends on adjust mode)
//      p = change adjust mode
//
//  Controls in record mode:
//  *follow serial prompts to record positions
//  mode:
//      y = change to manual mode
//
/*************************************************************************************************************************************************************************************************/

//unmodified
#include "mbed.h"                                       //mbed folder
#include "SDFileSystem.h"                               //SDFileSystem folder

//modified
#include "GPS.h"                                        //GPS folder
#include "PID.h"                                        //PID folder

//from scratch
#include "IMUDataAndFilters.h"
#include "navigation.h"
#include "Actuator.h"
#include "TrollingMotor.h"

//#define LIGHT_ON_PERIOD     1.0f
//#define LIGHT_OFF_PERIOD    5.0f

#define VOLT_MULT  (3.3f / (0.810f / (3.3f + 0.810f)))  //voltage divider 3.3k and 810 (VREF = 3.3V) keeps 12V measurements below 3.3V
#define RATIO_TOLERANCE     0.02f                       //How close the difference between the set ratio and current ratio before consider
#define MIN_RATIO           0.04f                       //Actuator hits retract limit swithc at 2.2%
#define MAX_RATIO           0.85f                       //Actuator hits extend limit switch at 87.6%
#define CENTER_RATIO        0.285f                      //Ratio where prop is centered

#define FIX             0                               // 2 = DGPS (more accurate but slower to initialize) 1 = GPS only (less accurate but faster to initialize)
#define ARRIVED         5.0f                            //Tolerance, in meters, for when goal location is reached
#define GPS_ACCUR       3.0f                            //accuracy of GPS unit
#define GPS_PERIOD      1.0f                            //GPS polling period (1 Hz)
#define GPS_POLL        0.5f
#define GPS_ALPHA       0.3f                            //GPS low pass alpha

#define RATE            0.3f                            //period of heading PID loop
#define headKc          1.0f                            //directly proportional
#define headTi          0.0f                            //a larger number makes the integral have less affect on the output
#define headTd          0.0f                            //a smaller number makes the derivative have less affect on the output

//PID/PID.cpp
PID headingPID(headKc, headTi, headTd, MAGN_PERIOD);    //Kc, Ti, Td, interval

//mbed classes (mbed folder)
Timer headingTime;
Timer acc;
Timer magn;
Timer inputTimer;
Timer loopTimer;
//Timer lightTimer;
AnalogIn battery(A0);                                   //analog input from +12v side of linear actuator potentiometer (uses voltage divider)
AnalogIn pot(A1);                                       //analog input from the wiper of linear actuator potentionmeter (uses voltage divider)
DigitalOut ldo(PTC10);                                  //Controls 3.3V LDO v-reg powering MTK3339 (GPS) 1 = GPS powered on and 0 = GPS powered down
//DigitalOut green(D3);                                   //Light output
//DigitalOut white(PTB9);                                 //Light output
Serial xBee(PTB11, PTB10);                              //UART 3 xbee serial
//end of mbed classes

//GPS/GPS.cpp
GPS gps(PTC15, PTC14);                                  //UART 4 GPS serial

//Not sure where "FILE" is defined
FILE *way;                                              //file pointer for waypoints on SD card
FILE *data;                                             //file pointer for datalog

//SDFileSystem/SDFileSystem.h
SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sd", PTE6, SDFileSystem::SWITCH_POS_NO, 50000000); //On board microSD

/***************************************************Prototype functions****************************************************************************************************************************/
void getDist(double posZero, double posOne, double curPos[2]);
void getAngle(double posZero, double posOne, double curPos[2], int flush);
/***************************************************End of prototype functions*********************************************************************************************************************/

/***************************************************Global Variables*******************************************************************************************************************************/
//TODO: rewrite polar vector functions to allow elimination of global variables

double polarVector[2]   =       {0,0.0000001f};         //poalarVector[0] = magnitude of vector, polarVector[1] = angle in degrees of vector

/*************************************************************************************************************************************************************************************************/
//                                                  MAIN
/*************************************************************************************************************************************************************************************************/

int main()
{    
    //int wayPtNum = 0, mode = 0, adjustMode = 0, lightCycle = 0;
    int wayPtNum = 0, mode = 0, adjustMode = 0;
    float magDiff = 0;
    float batVoltage = 0.0f, potVoltage = 0.0f, voltRatio = 0.0f;
    float curSet = 0.0f, prevSet = CENTER_RATIO;
    float filtered = 0.0000001f;
    double curPos[2]        =       {0,0};
    double goalPos[10][2];                                  //positions are initially read from SD card
                                                            //when a position is recorded in record mode, the previously stored position on the SD card is overwritten with the new position
                                                            
    //set the initial state of the lights
    //green = 1;
    //white = 0;
    
    //turn the GPS 3.3V LDO v-reg on
    ldo = 1;
    //wait for the GPS unit to boot
    wait(1);
    
    //set xBee baud rate
    xBee.baud(115200);
    xBee.printf("\nI'm Alive...\n");
    xBee.printf("Press any key to begin\n");
    
    //wait for keypress to begin
    while(!xBee.readable());
    
    //Get goal positions from SD card
    //start of SD card read
    way = fopen ("/sd/GPS_CORDS.txt", "rt");
    
    xBee.printf("Reading SD Card Please Wait\n");
    
    for(int x = 0; x<=9; x++)
    {
        fscanf(way, "%lf,%lf\n", &goalPos[x][0], &goalPos[x][1]);
        xBee.printf("waypoint %d = %lf,%lf\n", x, goalPos[x][0], goalPos[x][1]);
    }
    fclose(way);
    //end of SD card read
    
    //initialize magnetometer, accelerometer
    compass.init();
    wait(1);
    //Setup the GPS
    gps.Init();

    xBee.printf("gps initialized\n");
    
    xBee.printf("attempting to get a fix\n");
    
    while(gps.fixtype != FIX)
    {
        while(!gps.getData());
        if(gps.fixtype == 1)
        {
            xBee.printf("Waiting for DGPS fix\tcurrent fix = GPS only\n");
            xBee.printf("lat %f\tlon %f\thead %f\talt %f\tspd %f\tfix %d\tsat %d\n", gps.degLat, gps.degLon, gps.heading, gps.altitude, gps.speed, gps.fixtype, gps.satellites);
        }
        else xBee.printf("Waiting for DGPS fix\tcurrent fix = no fix\n");
    }
    
    //get IMU data and calculate the tilt compensated compass
    getAccel();                                     //IMUDataAndFilters.h
    getMagn();                                      //IMUDataAndFilters.h
    updateAngles();                                 //IMUDataAndFilters.h
    
    xBee.printf("lat %f\tlon %f\thead %f\talt %f\tspd %f\tfix %d\tsat %d\n", gps.degLat, gps.degLon, gps.heading, gps.altitude, gps.speed, gps.fixtype, gps.satellites);
    xBee.printf("dist %f\theading %f\n", polarVector[0], polarVector[1]);
    xBee.printf("\n\nstarting main loop\n");
        
    //set input constraints (heading difference)
    headingPID.setInputLimits(-180, 180);
    
    //set proportional output limits based on physical limits of actuator and mounting error
    float distFromCenter = calcEnds(CENTER_RATIO, MAX_RATIO, MIN_RATIO);        //navigation.h
    
    //set output constraints (linear actuator max and min)
    headingPID.setOutputLimits((CENTER_RATIO - distFromCenter), (CENTER_RATIO + distFromCenter));
    //set mode to auto
    headingPID.setMode(0);
    //We want the difference between actual heading and the heading needed to be zero
    headingPID.setSetPoint(0);
    
    //start timers
    loopTimer.start();
    headingTime.start();
    acc.start();
    magn.start();
    //lightTimer.start();
    while (1) 
    {
        /*********************************************************************************************************************************************************************************************/
        //                                          manual mode
        /*********************************************************************************************************************************************************************************************/

        if(mode == 0)
        {
            //checks to see if all three NEMA sentences from the GPS UART has been received
            gps.getData();
            
            //check timer
            if(loopTimer.read() > RATE)
            {
                //TODO: put this in a function
                
                //This moves actuator to the requested position
                //This is proportional feedback ONLY
                
                //read analog input from wiper on potentiometer on linear actuator
                potVoltage = pot.read();
                //read analog input from battery voltage
                batVoltage = battery.read();
                //calculate ratio of the two (using a ratio prevents battery voltage fluctuation from affecting actual position)
                voltRatio = potVoltage / batVoltage;
                
                //calculate the absolute value of how far off from requested actuator position we are (saves a few processor cycles)
                float absDiff = sqrt((prevSet - voltRatio) * (prevSet - voltRatio));
                //are we off enough to care?  if so, stop moving the actuator
                if(absDiff <= RATIO_TOLERANCE)
                {
                    turnStop(1.0f, 1.0f);
                    //xBee.printf("done\n");
                }
                
                //do we need to go further right?
                else if((prevSet - voltRatio) >= 0)
                {
                    turnRight(1.0f, 1.0f);
                    //xBee.printf("turning right\n");
                }
                
                //do we need to go further left?
                else
                {
                    turnLeft(1.0f, 1.0f);
                    //xBee.printf("turning left\n");
                }
                xBee.printf("battery = %f\tpot = %f\tratio = %f\tset %f\tHDOP = %f\tfix = %d\n", (batVoltage * VOLT_MULT), (potVoltage* VOLT_MULT), voltRatio, prevSet, gps.HDOP, gps.fixtype);
                
                //reset timer to zero
                loopTimer.reset();
            }
            
            //check to see if data is available on xBee
            if(xBee.readable())
            {
                
                //TODO: put this in a function
                
                char recChar = xBee.getc();
                
                //change to autonomous mode
                if(recChar == 'z')
                {
                    xBee.printf("Changing to autonomous mode\n");
                    goStop();
                    mode = 1;
                }
                //change to record mode
                else if(recChar == 'r')
                {
                    xBee.printf("Changing to record mode\n");
                    goStop();
                    mode = 3;
                }
                else if(recChar == '>')
                {
                    xBee.printf("Reseting system\n");
                    wait(0.5f);
                    NVIC_SystemReset();
                }
                else if(recChar == '<')
                {
                    xBee.printf("Power cycling GPS\nPlease wait\n");
                    goStop();
                    ldo = 0;
                    wait(0.5);
                    ldo = 1;
                    wait(0.5);
                }
                else if(recChar == '1')
                {
                    xBee.printf("stop\n");
                    goStop();
                }
                else if(recChar == 'w')
                {
                    goForward();
                    prevSet = CENTER_RATIO;
                    xBee.printf("Forward\n");
                }
                else if(recChar == 's')
                {
                    goBackward();
                    prevSet = CENTER_RATIO;
                    xBee.printf("backward\n");
                }
                else if(recChar == 'd')
                {
                    xBee.printf("large step right\n");
                    
                    //find the best step size since 0.1 step will go over the limit
                    if(prevSet + 0.1f > MAX_RATIO)
                    {
                        prevSet = prevSet + (MAX_RATIO - prevSet);
                    }
                    else
                    {
                        prevSet = prevSet + 0.1f;
                    }
                    xBee.printf("set = %f\n", prevSet);
                }
                //large step left
                else if(recChar == 'a')
                {
                    xBee.printf("large step left\n");
                    //find the best step size since 0.1 step will go over the limit
                    if(prevSet - 0.1f < MIN_RATIO)
                    {
                        prevSet = prevSet - (prevSet - MIN_RATIO);
                    }
                    else
                    {
                        prevSet = prevSet - 0.1f;
                    }
                    xBee.printf("set = %f\n", prevSet);
                }
                //small step right
                else if(recChar == 'e')
                {
                    xBee.printf("small step right\n");
                    if(prevSet + 0.01f > MAX_RATIO)
                    {
                        prevSet = prevSet + (MAX_RATIO - prevSet);
                    }
                    else
                    {
                        prevSet = prevSet + 0.01f;
                    }
                    xBee.printf("set = %f\n", prevSet);
                }
                else if(recChar == 'q')
                {
                    xBee.printf("Small step left\n");
                    if(prevSet - 0.01f < MIN_RATIO)
                    {
                        prevSet = prevSet - (prevSet - MIN_RATIO);
                    }
                    else
                    {
                        prevSet = prevSet - 0.01f;
                    }
                    xBee.printf("set = %f\n", prevSet);
                }
            }
            /*
            if(lightCycle == 0)
            {
                if(lightTimer.read() >= LIGHT_OFF_PERIOD)
                {
                    green = 0;
                    white = 0;
                    lightCycle = 1;
                    lightTimer.reset();
                }
            }
            else if(lightCycle == 1)
            {
                if(lightTimer.read() >= LIGHT_ON_PERIOD)
                {
                    green = 1;
                    white = 1;
                    lightCycle = 0;
                    lightTimer.reset();
                }
            }
            */
        }
        
        /*********************************************************************************************************************************************************************************************/
        //                                          autonomous mode
        /*********************************************************************************************************************************************************************************************/
        //default trolling motor state when entering autonomous mode is off (user must press "w" to go forward)
        if(mode == 1)
        {
            //check xBee
            if(xBee.readable())
            {
                //TODO: put this in a function
                
                char recChar = xBee.getc();
                //stop
                if(recChar == '1')
                {
                    xBee.printf("stop\n");
                    goStop();
                }
                //go forward
                if(recChar == 'w')
                {
                    xBee.printf("forward\n");
                    goForward();
                }
                //change to manual mode
                if(recChar == 'y')
                {
                    xBee.printf("Changing to manual mode\n");
                    goStop();
                    mode = 0;
                    wayPtNum = 0;
                }
                //increase calculated heading (use this to tweak/cheat calculated heading)
                else if(recChar == 'd')
                {
                    polarVector[1] = polarVector[1] + 1;
                    xBee.printf("increased angle %f\n", polarVector[1]);
                }
                //reduce calculated heading (use this to tweak/cheat calculated heading)
                else if(recChar == 'a')
                {
                    polarVector[1] = polarVector[1] - 1;
                    xBee.printf("reduced angle %f\n", polarVector[1]);
                }
                
                //increments settings based on adjust mode
                else if(recChar == '+')
                {
                    //adjust waypoint
                    if(adjustMode == 0)
                    {
                        if(wayPtNum != 9)
                        {
                            wayPtNum ++;
                            xBee.printf("waypoint increased to %d\n", wayPtNum);
                        }
                        else
                        {
                            xBee.printf("maximum waypoint reached\n");
                        }
                    }
                    //increment proportional gain of heading PID
                    else if(adjustMode == 1)
                    {
                        float curKc = headingPID.getPParam();
                        float curTi = headingPID.getIParam();
                        float curTd = headingPID.getDParam();
                        curKc = curKc + 0.1f;
                        headingPID.setTunings(curKc, curTi, curTd);
                        xBee.printf("Kc set to %f\n", curKc);
                    }
                    //increment integral gain of heading PID
                    else if(adjustMode == 2)
                    {
                        float curKc = headingPID.getPParam();
                        float curTi = headingPID.getIParam();
                        float curTd = headingPID.getDParam();
                        curTi = curTi + 0.1f;
                        headingPID.setTunings(curKc, curTi, curTd);
                        xBee.printf("Ti set to %f\n", curTi);
                    }
                    //increment derivative gain of heading PID
                    else if(adjustMode == 3)
                    {
                        float curKc = headingPID.getPParam();
                        float curTi = headingPID.getIParam();
                        float curTd = headingPID.getDParam();
                        curTd = curTd + 0.1f;
                        headingPID.setTunings(curKc, curTi, curTd);
                        xBee.printf("Td set to %f\n", curTd);
                    }
                }
                //decrements setting based on adjust mode
                else if(recChar == '-')
                {
                    if(adjustMode == 0)
                    {
                        if(wayPtNum != 0)
                        {
                            wayPtNum --;
                            xBee.printf("waypoint increased to %d\n", wayPtNum);
                        }
                        else
                        {
                            xBee.printf("minimum waypoint reached\n");
                        }
                    }
                    //decrement proportional gain of heading PID
                    else if(adjustMode == 1)
                    {
                        float curKc = headingPID.getPParam();
                        float curTi = headingPID.getIParam();
                        float curTd = headingPID.getDParam();
                        curKc = curKc - 0.1f;
                        headingPID.setTunings(curKc, curTi, curTd);
                        xBee.printf("Kc set to %f\n", curKc);
                    }
                    //decrement integral gain of heading PID
                    else if(adjustMode == 2)
                    {
                        float curKc = headingPID.getPParam();
                        float curTi = headingPID.getIParam();
                        float curTd = headingPID.getDParam();
                        curTi = curTi - 0.1f;
                        headingPID.setTunings(curKc, curTi, curTd);
                        xBee.printf("Ti set to %f\n", curTi);
                    }
                    //decrement derivative gain of heading PID
                    else if(adjustMode == 3)
                    {
                        float curKc = headingPID.getPParam();
                        float curTi = headingPID.getIParam();
                        float curTd = headingPID.getDParam();
                        curTd = curTd - 0.1f;
                        headingPID.setTunings(curKc, curTi, curTd);
                        xBee.printf("Td set to %f\n", curTd);
                    }
                }
                //change or reset current waypoint number
                else if(recChar == 'r')
                {
                    goStop();
                    //wayPtNum = 0;
                    //xBee.printf("waypoint count reset\n");
                    xBee.printf("Please enter desired waypoint (0-9)\t or press r to reset to zero\n");
                    while(!xBee.readable());
                    char tempWS[2];
                    tempWS[0] = xBee.getc();
                    
                    //press "r" again to reset waypoint number to zero
                    if(tempWS[0] == 'r')
                    {
                        wayPtNum = 0;
                    }
                    
                    //else enter the number of waypoint desired
                    else
                    {
                        sscanf(tempWS, "%d", &wayPtNum);
                        xBee.printf("waypoint is now %d\n", wayPtNum);
                    }
                }
                //change adjust mode
                else if(recChar == 'p')
                {
                    xBee.printf("To set adjust mode:\nEnter w to adjust waypoint number\nEnter c to adjust Kc\nEnter i to adjust Ti\nEnter d to adjust Td\nEnter z to exit\n");
                    while(!xBee.readable());
                    char recCharTemp = xBee.getc();
                    
                    //set adjust to edit waypoints
                    if(recCharTemp == 'w')
                    {
                        adjustMode = 0;
                    }
                    
                    //set adjust to edit proportional gain
                    else if(recCharTemp == 'c')
                    {
                        adjustMode = 1;
                        xBee.printf("Adjust mode set to Kc\tEnter + to increment and - to decrement Kc\n");
                    }
                    
                    //set adjust to edit integral gain
                    else if(recCharTemp == 'i')
                    {
                        adjustMode = 2;
                        xBee.printf("Adjust mode set to Ti\tEnter + to increment and - to decrement Ti\n");
                    }
                    
                    //set adjust to edit derivative gain
                    else if(recCharTemp == 'd')
                    {
                        adjustMode = 3;
                        xBee.printf("Adjust mode set to Td\tEnter + to increment and - to decrement Td\n");
                    }
                    //if any other key is pressed no change to adjust mode is made
                    else
                    {
                        xBee.printf("No changes made\n");
                    }
                }
            }
            //if no xBee data is received
            //else
            //{
                
                //TODO: put this in a function
                
                //checks to see if all three NEMA sentences from the GPS UART has been received
                if(gps.getData())
                {
                    double tempPos[2] = {0,0};
                    
                    //store most recent gps location in a temporary variable (using temporary variables because GPS data is accumulated in a low pass filter)
                    tempPos[0] = gps.degLat;
                    tempPos[1] = gps.degLon;
                    
                    //calculate the magnitude of the vector
                    getDist(goalPos[wayPtNum][0],goalPos[wayPtNum][1], tempPos);
                    
                    //calculate the angle of the vector
                    getAngle(goalPos[wayPtNum][0],goalPos[wayPtNum][1], tempPos, 0);
                    
                    //checks if the magnitude of distance from goal position is less than threshold for "arriving"
                    if(polarVector[0] <= ARRIVED)
                    {
                        xBee.printf("Goal Position %d reached!\n", wayPtNum);
                        wait(1);
                        
                        //increment waypoint number
                        wayPtNum ++;
                        
                        //we only have ten waypoints so we mus stop at ten
                        if(wayPtNum >= 10)
                        {
                            xBee.printf("Final Position Reached!\nShutting down\n");
                            goStop();
                            mode = 0;
                            //while(1);
                        }
                        else
                        {
                            //flush heading PID data since we have a new heading
                            headingPID.reset();
                            
                            //calculate the angle of the vector
                            getAngle(goalPos[wayPtNum ][0],goalPos[wayPtNum][1], goalPos[wayPtNum - 1], 1);
                            
                            xBee.printf("Moving to Goal Position %d\theading = %f\n", wayPtNum, polarVector[1]);
                        }
                    }
                }
                
                //time to read accelerometer?
                if(acc.read() >= ACCEL_PERIOD)
                {
                    //get accelerometer data
                    getAccel(); 
                    
                    //reset timer to zero
                    acc.reset();
                }
                
                //time to read magnatometer and calculate heading PID?
                if(magn.read() >= MAGN_PERIOD)
                {
                    //get magnatometer data
                    getMagn();                              //IMUDataAndFilters.h
                    updateAngles();                         //IMUDataAndFilters.h
                    filtered = lowPass(yaw, filtered, 0);   //IMUDataAndFilters.h
                    magDiff = whichWay(filtered, 0);        //navigation.h
                    
                    //give PID input
                    headingPID.setProcessValue(-magDiff);
                    
                    //get output from PID
                    curSet = headingPID.compute();
                    
                    //reset timer to zero
                    magn.reset();
                }
                             
                //This moves actuator to the requested position
                //This is proportional feedback only
                if(loopTimer.read() > RATE)
                {
                    //TODO: put this in a function
                    
                    //This moves actuator to the requested position
                    //This is proportional feedback ONLY
                    
                    //read analog input from wiper on potentiometer on linear actuator
                    potVoltage = pot.read();
                    //read analog input from battery voltage
                    batVoltage = battery.read();
                    //calculate ratio of the two (using a ratio prevents battery voltage fluctuation from affecting actual position)
                    voltRatio = potVoltage / batVoltage;
                    
                    //calculate the absolute value of how far off from requested actuator position we are (saves a few processor cycles)
                    float absDiff = sqrt((curSet - voltRatio) * (curSet - voltRatio));
                    
                    //are we off enough to care?  if so, stop moving the actuator
                    if(absDiff <= RATIO_TOLERANCE)
                    {
                        turnStop(1.0f, 1.0f);
                        //xBee.printf("done\n");
                    }
                    //do we need to turn right?
                    else if((curSet - voltRatio) >= 0)
                    {
                        if(voltRatio > MAX_RATIO)
                        {
                            //xBee.printf("Max Limit Reached\n");
                            turnStop(1.0f, 1.0f);
                        }
                        else
                        {
                            turnRight(1.0f, 1.0f);
                            //xBee.printf("turning Right\n");
                        }
                    }
                    //do we need to turn left?
                    else
                    {
                        if(voltRatio < MIN_RATIO)
                        {
                            //xBee.printf("Min Limit Reached\n");
                            turnStop(1.0f, 1.0f);
                        }
                        else
                        {
                            turnLeft(1.0f, 1.0f);
                            //xBee.printf("turning Left\n");
                        }
                    }
                    
                    xBee.printf("lat %f\tlon %f\thead %f\talt %f\tspd %f\tfix %d\tsat %d\n", gps.degLat, gps.degLon, gps.heading, gps.altitude, gps.speed, gps.fixtype, gps.satellites);
                    
                    //record data to SD card
                    data = fopen("/sd/data.txt", "a");
                    fprintf(data, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", wayPtNum, (batVoltage * VOLT_MULT), voltRatio, curSet, filtered, magDiff, polarVector[0], gps.degLat, gps.degLon, gps.heading, gps.altitude, gps.speed, gps.fixtype, gps.satellites);
                    fclose(data);
                    
                    //reset timer to zero
                    loopTimer.reset();
                }
            //}
        }
        
        /*********************************************************************************************************************************************************************************************/
        //                                          record position mode
        /*********************************************************************************************************************************************************************************************/
        
        if(mode == 3)
        {
            //stop the trolling motor
            goStop();
            
            xBee.printf("\nPlease enter position number (0-9), or press y to return to manual mode\n");
            
            while(!xBee.readable());
            char recChar = xBee.getc();
            recChar = xBee.getc();
            
            //return to manual mode
            if(recChar == 'y')
            {
                mode = 0;
            }
            else 
            {
                //TODO: put this in a function
                
                xBee.printf("\nFinding most accurate GPS position\nThis will take a few seconds\n\n");
            
                float lowestHDOP = 100;
                
                //take 50 GPS readings and keep the position with the lowest horizontal dilution of precision (HDOP)
                //lower HDOP = less error
                for(int i = 0; i< 50; i++)
                {
                    //wait for data to be available
                    //while(!gps._UltimateGps.readable())
                    gps.parseData();
                    
                    if(gps.HDOP <= lowestHDOP)
                    {
                        lowestHDOP = gps.HDOP;
                        curPos[0] = gps.degLat;
                        curPos[1] = gps.degLon;
                    }
                    xBee.printf("lat %f\tlon %f\thead %f\talt %f\tspd %f\tfix %d\tsat %d\n", gps.degLat, gps.degLon, gps.heading, gps.altitude, gps.speed, gps.fixtype, gps.satellites);
                    char tempChar = 'n';
                    
                    //first lockup was here
                    //now pressing "1" will break out of while loop
                    while(!gps.getData() && !(tempChar == '1'))
                    {
                        if(xBee.readable())
                        {
                            tempChar = xBee.getc();
                            i = 50;
                        }
                    }
                }
                if(recChar == '0')
                {
                    goalPos[0][0] = curPos[0];
                    goalPos[0][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);                    
                }
                else if(recChar == '1')
                {
                    goalPos[1][0] = curPos[0];
                    goalPos[1][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                else if(recChar == '2')
                {
                    goalPos[2][0] = curPos[0];
                    goalPos[2][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                else if(recChar == '3')
                {
                    goalPos[3][0] = curPos[0];
                    goalPos[3][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                else if(recChar == '4')
                {
                    goalPos[4][0] = curPos[0];
                    goalPos[4][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                else if(recChar == '5')
                {
                    goalPos[5][0] = curPos[0];
                    goalPos[5][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                else if(recChar == '6')
                {
                    goalPos[6][0] = curPos[0];
                    goalPos[6][1] = curPos[1];
                }
                else if(recChar == '7')
                {
                    goalPos[7][0] = curPos[0];
                    goalPos[7][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                else if(recChar == '8')
                {
                    goalPos[8][0] = curPos[0];
                    goalPos[8][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                else if(recChar == '9')
                {
                    goalPos[9][0] = curPos[0];
                    goalPos[9][1] = curPos[1];
                    
                    //write new coords to SD card
                    way = fopen("/sd/GPS_CORDS.txt", "w+");
                    for(int x = 0; x<=9; x++)
                    {
                        fprintf(way, "%2.10lf,%2.10lf\n", &goalPos[x][0], &goalPos[x][1]);
                    }
                    fclose(way);            
                }
                xBee.printf("position %c updated\t", recChar);
            }
            xBee.printf("returning to manual mode\n\n");
            mode = 0;
        }
    }
}

/*************************************************************************************************************************************************************************************************/
//                                                  create polar vector based on two sets of latitude and longitude
/*************************************************************************************************************************************************************************************************/
//TODO:
//getDist and getAngle need to be optimized
//they were one function but had to be hacked apart

void getDist(double posZero, double posOne, double curPos[2])
{
    double arcLength[2];
    double goalPos[2];
    goalPos[0] = posZero;
    goalPos[1] = posOne;
    
    /*Note: arc length = radius * angle*/
    //Y
    arcLength[1] = EARTHRADIUS * ((goalPos[0] - curPos[0]) * DEGREETORAD);
    //X
    arcLength[0] = EARTHRADIUS * ((goalPos[1] - curPos[1]) * DEGREETORAD);
    
    //calculate magnitude of vector
    polarVector[0] = sqrt((arcLength[0] * arcLength[0]) + (arcLength[1] * arcLength[1]));
}

void getAngle(double posZero, double posOne, double curPos[2], int flush)
{
    double tempAngle = 0;
    double arcLength[2];
    double goalPos[2];
    goalPos[0] = posZero;
    goalPos[1] = posOne;
    
    /*Note: arc length = radius * angle*/
    //Y
    arcLength[1] = EARTHRADIUS * ((goalPos[0] - curPos[0]) * DEGREETORAD);
    //X
    arcLength[0] = EARTHRADIUS * ((goalPos[1] - curPos[1]) * DEGREETORAD);
    
    //get rid of previous low passed angle data
    if(flush)
    {
        //Use arcTan(x/y) b/c we want our heading to be in respect to North (North = 0 degrees, East = 90 deg, etc.)
        polarVector[1] = (RADTODEGREE * (atan2(arcLength[0], arcLength[1]))); 
        //make negative angles positive
        if(polarVector[1] < 0) polarVector[1] = polarVector[1] + 360;    
    }
    
    //lowpass filter the angle
    else
    {
        
        tempAngle = (RADTODEGREE * (atan2(arcLength[0], arcLength[1])));
        
        if(tempAngle < 0) tempAngle = tempAngle + 360;
        polarVector[1] = lowPass(tempAngle, polarVector[1], 3);
    }
}