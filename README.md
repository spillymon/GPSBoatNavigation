Copyright [2015] [David R. Spillman]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.



Developed with mbed online compiler and exported as GCC (ARM Embedded)
mbed project: https://developer.mbed.org/users/Spilly/code/GPSNavigationNew/

  Created by: Ryan Spillman


  Last updated 4/29/2015

  This is the software for my teams autonomous boat that is our graduation/final project at Isothermal Community College 

  The user can drive the vehicle by sending chars over the xBee's serial connection
  GPS waypoints are stored on an external micro-SD card
  The user can record new waypoints to the SD card by driving to a location and entering record mode
  The user can also manually adjust the waypoints with a text editor

  A PID loop is used to control the heading of the vehicle

  The project uses a FRDM-K64f (Freescale microcontroller), a LSM303DLHC (magnetometer and accelerometer) to create a tilt compensated comapass, 
  MTK3339 GPS module, two xBee Pro S1, three TE KRPA-11 relays (relay logic H-bridge for trolling motor), and a L298n (H-Bridge for linear actuator)

!!!!!!!WARNING!!!!!!
  DO NOT update mbed or SDFileSystem folder
  Doing so will/may break this program
!!!!!!!WARNING!!!!!!

How To

  Requires a serial to usb adapter to connect an X-Bee to a PC
  Set both X-Bees up for 115200 baud
  Use TeraTerm (or other serial program) to read and send data over X-Bees
  Set TeraTerm new line character from CR (carrage return) to LF (line feed)
  Program starts by prompting user to press any key
  Once user presses a key, the program waits for a DGPS fix (can be set by changing "FIX")
  Once the program sees a DGPS fix, manual mode is enabled
  User can drive the vehicle in manual mode to any position
  User can record current position to a selected goal position in record mode
  In autonomous mode, the vehicle uses GPS data and compass data to navigate to each goal position

  Controls in manual mode:
      directional:
          w = forward
          s = backward
          a = left
          d = right
      mode:
          r = change to record mode
          z = change to autonomous mode

  Controls in autonomous mode:
  mode:
      y = change to manual mode
  adjustments:
      d = increase angle
      a = decrease angle
      r = enter new waypoint number
      + = increase (depends on adjust mode)
      - = decrease (depends on adjust mode)
      p = change adjust mode

  Controls in record mode:
  *follow serial prompts to record positions
  mode:
      y = change to manual mode
