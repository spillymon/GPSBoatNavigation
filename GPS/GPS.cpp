/**************************************************************************************************************************************************************
//  This is a modified version of mbed /users/SamClarke/code/GPS/ for MTK3339 GPS module
//
//  Changes made by Ryan Spillman:
//
//  Last updated: 4/29/2015
//
//  added commands to the initialization function to set baud rate, set update rate, enabled DGPS WAAS, and set minimum navigation speed
//  added a function to put the unit into sleep mode
//  added several zeros after the decimal places to fix weird rounding errors
//  added information about checksums
//  Modified parse data function to get all of the NMEA sentences at once.  Previously it only grabbed one sentence each time the function was called
//  Changed latitude and longitude calculation variables to doubles to decrease math errors
//  Added function to get GPGSA NMEA sentence (we have HDOP!)
//
//  IMPORTANT:
//  Any changes made to commands sent to GPS unit require corresponding changes to the
//  checksum at the end of the command
//  Here is a tool for calculating the checksum: 
//  http://siliconsparrow.com/demos/nmeachecksum.php
**************************************************************************************************************************************************************/

#include "GPS.h"
GPS::GPS(PinName tx, PinName rx) : _UltimateGps(tx, rx)
{
    //_UltimateGps.baud(9600);  //use this for default MTK3339 adafruit firmware
    _UltimateGps.baud(38400);  //Changed firmware of MTK3339 to 38400 baud with update rate of 5Hz (GlobalTopFlashTool to change firmware)
}
void GPS::coldStart()
{
    _UltimateGps.printf("$PMTK103*30\r\n");
}
int GPS::getGPGGA()
{
    if(sscanf(NEMA, "GPGGA, %*f, %*f, %*c, %*f, %*c, %d, %d, %*f, %f", &fixtype, &satellites, &altitude) >=1)
    {
            //if(fixtype == 0) return 0;
            return 1;
    }
    return 0;
}
int GPS::getGPGSA()
{
        if(satellites == 4)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*c, %*c, %*c, %*c,,,,,,,,,%f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else if(satellites == 5)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d,,,,,,,, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else if(satellites == 6)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d, %*d,,,,,,, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else if(satellites == 7)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d, %*d, %*d,,,,,, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else if(satellites == 8)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d,,,,, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else if(satellites == 9)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d,,,, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else if(satellites == 10)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d,,, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else if(satellites == 11)
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d,, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        else
        {
            if(sscanf(NEMA, "GPGSA, %c, %d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %*d, %f, %f, %f", &mode, &status, &PDOP, &HDOP, &VDOP) >=1) return 1;
        }
        return 0;
}
int GPS::getGPRMC()
{
    
    if(sscanf(NEMA, "GPRMC, %2d%2d%f, %c, %f, %c, %f, %c, %f, %f, %2d%2d%2d", &hours, &minutes, &seconds, &validity, &latitude, &ns, &longitude, &ew, &speed, &heading, &day, &month, &year) >=1)
    {
        year += 2000;
        if(ns =='S') {
            latitude   *= -1.00000000000000000f;
        }
        if(ew =='W') {
            longitude  *= -1.00000000000000000f;
        }
        double degrees = trunc(latitude / 100.0000000000000000f);
        double minutes = latitude - (degrees * 100.00000000000000000f);
        degLat = degrees + minutes / 60.00000000000000000f;
        degrees = trunc(longitude / 100.0000000000000000f);
        minutes = longitude - (degrees * 100.00000000000000000f);
        degLon = degrees + minutes / 60.00000000000000000f;
        if(fixtype == 1) {
            fix = "Positive";
        }
        if(fixtype == 2) {
            fix = "Differential";
        }
        if(heading > 0.00f && heading < 45.00f) {
            cardinal = "NNE";
        }
        if(heading == 45.00f) {
            cardinal = "NE";
        }
        if(heading > 45.00f && heading < 90.00f) {
            cardinal = "ENE";
        }
        if(heading == 90.00f) {
            cardinal = "E";
        }
        if(heading > 90.00f && heading < 135.00f) {
            cardinal = "ESE";
        }
        if(heading == 135.00f) {
            cardinal = "SE";
        }
        if(heading > 135.00f && heading < 180.00f) {
            cardinal = "SSE";
        }
        if(heading == 180.00f) {
            cardinal = "S";
        }
        if(heading > 180.00f && heading < 225.00f) {
            cardinal = "SSW";
        }
        if(heading == 225.00f) {
            cardinal = "SW";
        }
        if(heading > 225.00f && heading < 270.00f) {
            cardinal = "WSW";
        }
        if(heading == 270.00f) {
            cardinal = "W";
        }
        if(heading > 270.00f && heading < 315.00f) {
            cardinal = "WNW";
        }
        if(heading == 315.00f) {
            cardinal = "NW";
        }
        if(heading > 315.00f && heading < 360.00f) {
            cardinal = "NNW";
        }
        if(heading == 360.00f || heading == 0.00f) {
            cardinal = "N";
        }
        kph = speed*1.852f;
        return 1;
    }
    return 0;
}
int GPS::parseData()
{
    if(getGPGGA()) 
    {
        sentenceCount = sentenceCount + 1;
        return 1;
    }
    else if(getGPRMC())
    {
        sentenceCount = sentenceCount + 1;
        return 1;
    }
    else if(getGPGSA())
    {
        sentenceCount = sentenceCount + 1;
        return 1;
    }
    return 0;
}


float GPS::trunc(float v)
{
    if(v < 0.0f) {
        v*= -1.0f;
        v = floor(v);
        v*=-1.0f;
    } else {
        v = floor(v);
    }
    return v;
}

bool GPS::getData()
{
    if(!_UltimateGps.readable()) return 0;
    char tempChar = _UltimateGps.getc();
    //printf("%c\n", tempChar);
    if(tempChar == '$')
    {
        for(int i=0; i<256; i++) 
        {
            NEMA[i] = _UltimateGps.getc();
            if(NEMA[i] == '\r') 
            {
                NEMA[i] = 0;
                parseData();
                if(_UltimateGps.readable())
                {
                    
                    tempChar = _UltimateGps.getc();
                    if(tempChar != '$')
                    {
                        tempChar = _UltimateGps.getc();
                    }
                    if(tempChar == '$')
                    {
                        for(int i=0; i<256; i++) 
                        {
                            NEMA[i] = _UltimateGps.getc();
                            if(NEMA[i] == '\r') 
                            {
                                NEMA[i] = 0;
                                parseData();
                                if(_UltimateGps.readable())
                                {
                                    tempChar = _UltimateGps.getc();
                                    if(tempChar != '$')
                                    {
                                        tempChar = _UltimateGps.getc();
                                    }
                                    if(tempChar == '$')
                                    {
                                        for(int i=0; i<256; i++) 
                                        {
                                            NEMA[i] = _UltimateGps.getc();
                                            if(NEMA[i] == '\r') 
                                            {
                                                NEMA[i] = 0;
                                                parseData();
                                                if(sentenceCount == 3)
                                                {
                                                    sentenceCount = 0;
                                                    return 1;
                                                }
                                                return 0;
                                            }
                                        }
                                    }
                                    
                                    else 
                                    {
                                        return 0;
                                    }
                                }
                                if(sentenceCount == 3)
                                {
                                    sentenceCount = 0;
                                    return 1;
                                }
                                return 0;
                            }       
                        }
                    }
                }
                if(sentenceCount == 3)
                {
                    sentenceCount = 0;
                    return 1;
                }
                return 0;
            }
        }
        
    }
    return 0;
}

void GPS::Init()
{
    //don't need to send baud rate command because it is already set to 38400 in firmware
    //_UltimateGps.printf("$PMTK251,38400*27\r\n");                                     //set baud (any higher and the serial buffer overflows)
    
    //Note: SBAS can only be enabled when update rate is less than or equal to 5Hz.
    //_UltimateGps.printf("$PMTK220,100*2F\r\n");                                       //10 Hz update
    //_UltimateGps.printf("$PMTK220,200*2C\r\n");                                       //5 Hz udpate
    _UltimateGps.printf("$PMTK220,1000*1f\r\n");                                        //1 Hz udpate
    
    _UltimateGps.printf("$PMTK225,0*2bt\r\n");                                          //disable always locate (datasheet indicates that this negatively affects accuracy)
    _UltimateGps.printf("$PMTK301,2*2et\r\n");                                          //set DGPS to use WAAS
    
    //_UltimateGps.printf("$PMTK386,0.8*35\r\n");                                       //set Nav Speed threshold to 0.8 m/s
    //_UltimateGps.printf("$PMTK386,0.2*3f\r\n");                                       //set Nav Speed threshold to 0.2 m/s
    _UltimateGps.printf("$PMTK386,0*23\r\n");                                           //disable Nav Speed threshold
    
    //_UltimateGps.printf("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");     //GPRMC and GPGGA
    //_UltimateGps.printf("$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");     //GPRMC and GPGSA
    _UltimateGps.printf("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");       //GPRMC, GPGGA, and GPGSA
    //_UltimateGps.attach(this, &GPS::GPSInt);
}

void GPS::Sleep(int sleep)
{
    if(sleep == 1)
    {
        _UltimateGps.printf("$$PMTK161,0*28\r\n");  //go to sleep
    }
    else
    {
        _UltimateGps.printf("$$PMTK161,1*28\r\n");          //wake up
        
        /*unit goes back to default config when put to sleep*/
            
        //Note: SBAS can only be enabled when update rate is less than or equal to 5Hz.
        //_UltimateGps.printf("$PMTK220,100*2F\r\n");                                       //10 Hz update
        //_UltimateGps.printf("$PMTK220,200*2C\r\n");                                       //5 Hz udpate
        _UltimateGps.printf("$PMTK220,1000*1f\r\n");                                        //1 Hz udpate
        
        _UltimateGps.printf("$PMTK225,0*2bt\r\n");                                          //disable always locate (datasheet indicates that this negatively affects accuracy)
        _UltimateGps.printf("$PMTK301,2*2et\r\n");                                          //set DGPS to use WAAS
        
        //_UltimateGps.printf("$PMTK386,0.8*35\r\n");                                       //set Nav Speed threshold to 0.8 m/s
        //_UltimateGps.printf("$PMTK386,0.2*3f\r\n");                                       //set Nav Speed threshold to 0.2 m/s
        _UltimateGps.printf("$PMTK386,0*23\r\n");                                           //disable Nav Speed threshold
        
        //_UltimateGps.printf("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");     //GPRMC and GPGGA
        //_UltimateGps.printf("$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");     //GPRMC and GPGSA
        _UltimateGps.printf("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");       //GPRMC, GPGGA, and GPGSA
    }
}
