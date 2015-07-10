/*************************************************************************************************************************************************************/
//  Created by: Ryan Spillman
//
//  Last updated 4/29/2015
//
//  Contains function that outputs difference in compass heading(magHead) and heading required(calcHead)
/*************************************************************************************************************************************************************/

#define navigation_h

//Tolerance for heading actual and heading needed
#define HEADDIFF 0.000000000000000000f

/*************************************************************************************************************************************************************/
//                                                      whichWay
//
//  Outputs difference in compass heading(magHead) and heading required(calcHead)
//  negative is left and positive is right
/*************************************************************************************************************************************************************/
//TODO: Check for redundancy
float whichWay(float magHead, float calcHead)
{
    float magDiff;
    
    float absOfDiff = sqrt((calcHead - magHead) * (calcHead - magHead));
    
    //Is the heading off enough to care?
    if((absOfDiff >= HEADDIFF) && (absOfDiff <= (360 - HEADDIFF)))
    //if(1)
    {
        //quadrant I
        if(calcHead < 90)
        {
            if(calcHead < magHead)
            {
                if(magHead < (180 + calcHead))
                {
                    //turn left need negative
                    magDiff = calcHead - magHead;
                }
                else
                {
                    //turn right need positive
                    magDiff = calcHead - magHead + 360;
                }
            }
            else
            {
                if(magHead < (180 + calcHead))
                {
                    //turn left need negative
                    magDiff = calcHead - magHead;
                }
                else
                {
                    //turn right need positive
                    magDiff = calcHead - magHead + 360;
                }
            }
        }
        //quadrant II
        else if(calcHead < 180)
        {
            if(calcHead < magHead)
            {
                if(magHead < (180 + calcHead))
                {
                    //turn left need negative
                    magDiff = calcHead - magHead;
                }
                else
                {
                    //turn right need positive
                    magDiff = calcHead - magHead + 360;
                }
            }
            else
            {
                if(magHead < (180 + calcHead))
                {
                    //turn left need negative
                    magDiff = calcHead - magHead;
                }
                else
                {
                    //turn right need positive
                    magDiff = calcHead - magHead + 360;
                }
            }
        }
        //quadrant III
        else if(calcHead < 270)
        {
            if(calcHead < magHead)
            {
                if(magHead < (180 + calcHead))
                {
                    //turn left need negative
                    magDiff = calcHead - magHead;
                }
                else
                {
                    //turn right need positive
                    magDiff = calcHead - magHead + 360;
                }
            }
            else
            {
                /*
                if(magHead < (180 + calcHead))
                {
                    //turn left need negative
                    magDiff = calcHead - magHead;
                }
                else
                {
                
                    //turn right need positive
                    magDiff = calcHead - magHead + 360;
                }
                */
                magDiff = calcHead - magHead - 90;
            }
        }
        //quadrant IV
        else
        {
            
            if(calcHead < magHead)
            {
                magDiff = calcHead - magHead;
            }
            else
            {
                if(magHead < (calcHead - 180))
                {
                    
                    magDiff = calcHead - 360 - magHead;
                }
                else
                {
                    //turn right need positive
                    magDiff = calcHead - magHead;
                }
            }
        }
        return magDiff;
    }
    return 0;
}
