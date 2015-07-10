/*************************************************************************************************************************************************************/
//  Created by: Ryan Spillman
//
//  Last updated 4/29/2015
//
//  Contains functions for controlling trolling motor relays
/*************************************************************************************************************************************************************/

//mbed classes (mbed folder)
DigitalOut direction(D5);
DigitalOut enable(D4);

/*************************************************************************************************************************************************************/
//                                                      Relay Trolling Motor Control
/*************************************************************************************************************************************************************/

int prevState = 0;

void goForward()
{
    //Are we changing states?
    if(prevState == 2)
    {
        //Short to ground will occur if parallel H-Bridge Relays are not synchronous when changing states
        //turn off Run/Stop relay first to prevent short
        enable = 0;
        wait(0.5);
        enable = 1;
        prevState = 1;
    }
    else
    {
        enable = 1;
        direction = 0;
    }
}

void goBackward()
{
    //Are we changing states?
    if(prevState == 1)
    {
        //Short to ground will occur if parallel H-Bridge Relays are not synchronous when changing states
        //turn off Run/Stop relay first to prevent short
        enable = 0;
        wait(0.5);
        enable = 1;
        prevState = 2;
    }
    else
    {
        enable = 1;
        direction = 1;
    }
}

void goStop()
{
    enable = 0;
}