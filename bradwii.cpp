/* 
Copyright 2013 Brad Quick

Some of this code is based on Multiwii code by Alexandre Dubus (www.multiwii.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*

The code is for controlling multi-copters.  Many of the ideas in the code come from the Multi-Wii project
(see multiwii.com).  This project doesn't contain all of the features in Multi-Wii, but I believe it incorporates
a number of improvements.

In order to make the code run quickly on 8 bit processors, much of the math is done using fixed point numbers
instead of floating point.  Much pain was taken to write almost the entire code without performing any
division, which is slow. As a result, main loop cycles can take well under 2 milliseconds.

A second advantage is that I believe that this code is more logically layed out and better commented than 
some other multi-copter code.  It is designed to be easy to follow for the guy who wants to understand better how
the code works and maybe wants to modify it.

In general, I didn't include code that I haven't tested myself, therefore many aircraft configurations, control boards,
sensors, etc. aren't yet included in the code.  It should be fairly easy, however for interested coders to add the
components that they need.

If you find the code useful, I'd love to hear from you.  Email me at the address that's shown vertically below:

b         I made my
r         email address
a         vertical so
d         the spam bots
@         won't figure
j         it out.
a         - Thanks.
m
e
s
l
t
a
y
l
o
r
.
c
o
m

*/

#include "stdio.h"
#include <avr/io.h>
#include <avr/interrupt.h> 

// library headers
#include "lib_timers.h"
#include "lib_serial.h"
#include "lib_i2c.h"
#include "lib_digitalio.h"
#include "lib_pwm.h"
#include "lib_fp.h"

// project file headers
#include "bradwii.h"
#include "rx.h"
#include "serial.h"
#include "output.h"
#include "gyro.h" 
#include "accelerometer.h"
#include "imu.h"
#include "baro.h"
#include "compass.h"
#include "eeprom.h"
#include "gps.h"
#include "navigation.h"
#include "pilotcontrol.h"
#include "autotune.h"

globalstruct global; // global variables
usersettingsstruct usersettings; // user editable variables

fixedpointnum altitudeHoldDesiredAltitude;
fixedpointnum integratedAltitudeError; // for pid control

fixedpointnum integratedAngleError[3];

// limit pid windup
#define INTEGRATEDANGLEERRORLIMIT FIXEDPOINTCONSTANT(5000) 

// timesliver is a very small slice of time (.002 seconds or so).  This small value doesn't take much advantage
// of the resolution of fixedpointnum, so we shift timesliver an extra TIMESLIVEREXTRASHIFT bits.
unsigned long timeslivertimer=0;

// It all starts here:
int main(void) {
   // start with default user settings in case there's nothing in eeprom
   default_user_settings();
   // try to load usersettings from eeprom
   read_user_settings_from_eeprom();
   
   // set our LED as a digital output
   lib_digitalio_initpin(LED1_OUTPUT,DIGITALOUTPUT);

   //initialize the libraries that require initialization
   lib_timers_init();
   lib_i2c_init();

   // pause a moment before initializing everything. To make sure everything is powered up
   lib_timers_delaymilliseconds(100);
   
   // initialize all other modules
   init_rx();
   init_outputs();
   serial_init();   
   init_gyro();
   init_acc();
   init_baro();
   init_compass();
   init_gps();
   init_imu();
   
   // set the default i2c speed to 400 KHz.  If a device needs to slow it down, it can, but it should set it back.
   lib_i2c_setclockspeed(I2C_400_KHZ);

   global.armed=0;
   global.navigationMode=NAVIGATION_MODE_OFF;
   global.failsafeTimer=lib_timers_starttimer();

   for(;;) {
      // check to see what switches are activated
      check_checkbox_items();
      
      // check for config program activity
      serial_check_for_action();   
      
      calculate_timesliver();
      
      // run the imu to estimate the current attitude of the aircraft
      imu_calculate_estimated_attitude();

      // arm and disarm via rx aux switches
      if (global.rxValues[THROTTLE_INDEX]<FPSTICKLOW) { // see if we want to change armed modes
          if (!global.armed) {
             if (global.activeCheckboxItems & CHECKBOX_MASK_ARM) {
                 global.armed=1;
                #if (GPS_TYPE!=NO_GPS)
                 navigation_set_home_to_current_location();
                #endif
                 global.headingWhenArmed=global.currentEstimatedEulerAttitude[YAW_INDEX];
                 global.altitudeWhenArmed=global.baroRawAltitude;
             }
          } else if (!(global.activeCheckboxItems & CHECKBOX_MASK_ARM)) global.armed=0;
      }

      #if (GPS_TYPE!=NO_GPS)
      // turn on or off navigation when appropriate
      if (global.navigationMode==NAVIGATION_MODE_OFF) {
          if (global.activeCheckboxItems & CHECKBOX_MASK_RETURNTOHOME) { // return to home switch turned on
              navigation_set_destination(global.gpsHomeLatitude,global.gpsHomeLongitude);
              global.navigationMode=NAVIGATION_MODE_RETURN_TO_HOME;
          } else if (global.activeCheckboxItems & CHECKBOX_MASK_POSITIONHOLD) { // position hold turned on
              navigation_set_destination(global.gpsCurrentLatitude,global.gpsCurrentLongitude);
              global.navigationMode=NAVIGATION_MODE_POSITION_HOLD;
          }
      } else { // we are currently navigating
          // turn off navigation if desired
          if ((global.navigationMode==NAVIGATION_MODE_RETURN_TO_HOME && !(global.activeCheckboxItems & CHECKBOX_MASK_RETURNTOHOME)) || (global.navigationMode==NAVIGATION_MODE_POSITION_HOLD && !(global.activeCheckboxItems & CHECKBOX_MASK_POSITIONHOLD))) {
              global.navigationMode=NAVIGATION_MODE_OFF;
            
              // we will be turning control back over to the pilot.
              reset_pilot_control();
          }
      }
        #endif
      
       // read the receiver
       read_rx();
      
       // turn on the LED when we are stable and the gps has 5 satelites or more
      #if (GPS_TYPE==NO_GPS)
       lib_digitalio_setoutput(LED1_OUTPUT, (global.stable==0)==LED1_ON);
      #else
       lib_digitalio_setoutput(LED1_OUTPUT, (!(global.stable && global.gpsNumSatelites>=5))==LED1_ON);
      #endif
      
       // get the angle error.  Angle error is the difference between our current attitude and our desired attitude.
       // It can be set by navigation, or by the pilot, etc.
       fixedpointnum angleError[3];
      
       // let the pilot control the aircraft.
       get_angle_error_from_pilot_input(angleError);
      
#if (GPS_TYPE!=NO_GPS)
       // read the gps
       unsigned char gotNewGpsReading=read_gps();

       // if we are navigating, use navigation to determine our desired attitude (tilt angles)
       if (global.navigationMode!=NAVIGATION_MODE_OFF) { // we are navigating
           navigation_set_angle_error(gotNewGpsReading,angleError);
       }
#endif

       if (global.rxValues[THROTTLE_INDEX]<FPSTICKLOW) {
           // We are probably on the ground. Don't accumnulate error when we can't correct it
           reset_pilot_control();
         
           // bleed off integrated error by averaging in a value of zero
           lib_fp_lowpassfilter(&integratedAngleError[ROLL_INDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
           lib_fp_lowpassfilter(&integratedAngleError[PITCH_INDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
           lib_fp_lowpassfilter(&integratedAngleError[YAW_INDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
       }

#ifndef NO_AUTOTUNE
       // let autotune adjust the angle error if the pilot has autotune turned on
       if (global.activeCheckboxItems & CHECKBOX_MASK_AUTOTUNE) {
           if (!(global.previousActiveCheckboxItems & CHECKBOX_MASK_AUTOTUNE)) {
               autotune(angleError,AUTOTUNESTARTING); // tell autotune that we just started autotuning
           } else {
               autotune(angleError,AUTOTUNETUNING); // tell autotune that we are in the middle of autotuning
           }
       } else if (global.previousActiveCheckboxItems & CHECKBOX_MASK_AUTOTUNE) {
           autotune(angleError,AUTOTUNESTOPPING); // tell autotune that we just stopped autotuning
       }
#endif

       // get the pilot's throttle component
       // convert from fixedpoint -1 to 1 to fixedpoint 0 to 1
       fixedpointnum throttleOutput=(global.rxValues[THROTTLE_INDEX]>>1)+FIXEDPOINTCONSTANT(.5)+FPTHROTTLETOMOTOROFFSET;

       // keep a flag to indicate whether we shoud apply altitude hold.  The pilot can turn it on or
       // uncrashability mode can turn it on.
       unsigned char altitudeHoldActive=0;
      
       if (global.activeCheckboxItems & CHECKBOX_MASK_ALTHOLD) {
           altitudeHoldActive=1;
           if (!(global.previousActiveCheckboxItems & CHECKBOX_MASK_ALTHOLD)) { // we just turned on alt hold.  Remember our current alt. as our target
               altitudeHoldDesiredAltitude=global.altitude;
               integratedAltitudeError=0;
           }
       }
       
      // uncrashability mode
      #define UNCRASHABLELOOKAHEADTIME FIXEDPOINTONE // look ahead one second to see if we are going to be at a bad altitude
      #define UNCRASHABLERECOVERYANGLE FIXEDPOINTCONSTANT(15) // don't let the pilot pitch or roll more than 20 degrees when altitude is too low.
      #define FPUNCRASHABLE_RADIUS FIXEDPOINTCONSTANT(UNCRAHSABLE_RADIUS) 
      #define FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET FIXEDPOINTCONSTANT(UNCRAHSABLE_MAX_ALTITUDE_OFFSET)
#if (GPS_TYPE!=NO_GPS)
       // keep a flag that tells us whether uncrashability is doing gps navigation or not
       static unsigned char doingUncrashableNavigationFlag;
#endif
       // we need a place to remember what the altitude was when uncrashability mode was turned on
       static fixedpointnum uncrashabilityMinimumAltitude;
       static fixedpointnum uncrasabilityDesiredAltitude;
       static unsigned char doingUncrashableAltitudeHold=0;
      
       if (global.activeCheckboxItems & CHECKBOX_MASK_UNCRASHABLE) { // uncrashable mode
           // First, check our altitude
           // are we about to crash?
           if (!(global.previousActiveCheckboxItems & CHECKBOX_MASK_UNCRASHABLE)) { // we just turned on uncrashability.  Remember our current altitude as our new minimum altitude.
               uncrashabilityMinimumAltitude=global.altitude;
#if (GPS_TYPE!=NO_GPS)
               doingUncrashableNavigationFlag=0;
               // set this location as our new home
               navigation_set_home_to_current_location();
#endif
           }
         
           // calculate our projected altitude based on how fast our altitude is changing
           fixedpointnum projectedAltitude=global.altitude+lib_fp_multiply(global.altitudeVelocity,UNCRASHABLELOOKAHEADTIME);
         
           if (projectedAltitude>uncrashabilityMinimumAltitude+FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET) { // we are getting too high
               // Use Altitude Hold to bring us back to the maximum altitude.
               altitudeHoldDesiredAltitude=uncrashabilityMinimumAltitude+FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET;
               integratedAltitudeError=0;
               altitudeHoldActive=1;
           } else if (projectedAltitude<uncrashabilityMinimumAltitude) { // We are about to get below our minimum crashability altitude
               if (doingUncrashableAltitudeHold==0) { // if we just entered uncrashability, set our desired altitude to the current altitude
                   uncrasabilityDesiredAltitude=global.altitude;
                   integratedAltitudeError=0;
                   doingUncrashableAltitudeHold=1;
               }
            
               // don't apply throttle until we are almost level
               if (global.estimatedDownVector[Z_INDEX]>FIXEDPOINTCONSTANT(.4)) {
                   altitudeHoldDesiredAltitude=uncrasabilityDesiredAltitude;
                   altitudeHoldActive=1;
               } else throttleOutput=0; // we are trying to rotate to level, kill the throttle until we get there

               // make sure we are level!  Don't let the pilot command more than UNCRASHABLERECOVERYANGLE
               lib_fp_constrain(&angleError[ROLL_INDEX],-UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[ROLL_INDEX],UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[ROLL_INDEX]);
               lib_fp_constrain(&angleError[PITCH_INDEX],-UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[PITCH_INDEX],UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[PITCH_INDEX]);
           } else doingUncrashableAltitudeHold=0;
         
#if (GPS_TYPE!=NO_GPS)
           // Next, check to see if our GPS says we are out of bounds
           // are we out of bounds?
           fixedpointnum bearingFromHome;
           fixedpointnum distanceFromHome=navigation_getdistanceandbearing(global.gpsCurrentLatitude,global.gpsCurrentLongitude,global.gpsHomeLatitude,global.gpsHomeLongitude,&bearingFromHome);
         
           if (distanceFromHome>FPUNCRASHABLE_RADIUS) { // we are outside the allowable area, navigate back toward home
               if (!doingUncrashableNavigationFlag) { // we just started navigating, so we have to set the destination
                   navigation_set_destination(global.gpsHomeLatitude,global.gpsHomeLongitude);
                   doingUncrashableNavigationFlag=1;
               }
               
               // Let the navigation figure out our roll and pitch attitudes
               navigation_set_angle_error(gotNewGpsReading,angleError);
           }
           else doingUncrashableNavigationFlag=0;
#endif
         }
#if (GPS_TYPE!=NO_GPS)
       else doingUncrashableNavigationFlag=0;
#endif

#if (BAROMETER_TYPE!=NO_BAROMETER)
       // check for altitude hold and adjust the throttle output accordingly
       if (altitudeHoldActive) {
           integratedAltitudeError+=lib_fp_multiply(altitudeHoldDesiredAltitude-global.altitude,global.timesliver);
           lib_fp_constrain(&integratedAltitudeError,-INTEGRATEDANGLEERRORLIMIT,INTEGRATEDANGLEERRORLIMIT); // don't let the integrated error get too high
         
           // do pid for the altitude hold and add it to the throttle output
           throttleOutput+=lib_fp_multiply(altitudeHoldDesiredAltitude-global.altitude,usersettings.pid_pgain[ALTITUDE_INDEX])-lib_fp_multiply(global.altitudeVelocity,usersettings.pid_dgain[ALTITUDE_INDEX])+lib_fp_multiply(integratedAltitudeError,usersettings.pid_igain[ALTITUDE_INDEX]);
       }
#endif
       if ((global.activeCheckboxItems & CHECKBOX_MASK_AUTOTHROTTLE) || altitudeHoldActive) {
           // Auto Throttle Adjust - Increases the throttle when the aircraft is tilted so that the vertical
           // component of thrust remains constant.
           // The AUTOTHROTTLEDEADAREA adjusts the value at which the throttle starts taking effect.  If this
           // value is too low, the aircraft will gain altitude when banked, if it's too low, it will lose
           // altitude when banked. Adjust to suit.
            #define AUTOTHROTTLEDEADAREA FIXEDPOINTCONSTANT(.25)

           if (global.estimatedDownVector[Z_INDEX]>FIXEDPOINTCONSTANT(.3)) {
               // Divide the throttle by the throttleOutput by the z component of the down vector
               // This is probaly the slow way, but it's a way to do fixed point division
               fixedpointnum recriprocal=lib_fp_invsqrt(global.estimatedDownVector[Z_INDEX]);
               recriprocal=lib_fp_multiply(recriprocal,recriprocal);
         
               throttleOutput=lib_fp_multiply(throttleOutput-AUTOTHROTTLEDEADAREA,recriprocal)+AUTOTHROTTLEDEADAREA;
           }
       }

       // if we don't hear from the receiver for over a second, try to land safely
       if (lib_timers_gettimermicroseconds(global.failsafeTimer)>1000000L) {
           throttleOutput=FPFAILSAFEMOTOROUTPUT;

           // make sure we are level!
           angleError[ROLL_INDEX]=-global.currentEstimatedEulerAttitude[ROLL_INDEX];
           angleError[PITCH_INDEX]=-global.currentEstimatedEulerAttitude[PITCH_INDEX];
       }

       // calculate output values.  Output values will range from 0 to 1.0

       // calculate pid outputs based on our angleErrors as inputs
       fixedpointnum pidoutput[3];
      
       // Gain Scheduling essentialy modifies the gains depending on
       // throttle level. If GAIN_SCHEDULING_FACTOR is 1.0, it multiplies PID outputs by 1.5 when at full throttle,
       // 1.0 when at mid throttle, and .5 when at zero throttle.  This helps
       // eliminate the wobbles when decending at low throttle.
       fixedpointnum gainschedulingmultiplier=lib_fp_multiply(throttleOutput-FIXEDPOINTCONSTANT(.5),FIXEDPOINTCONSTANT(GAIN_SCHEDULING_FACTOR))+FIXEDPOINTONE;
      
       for (int x=0;x<3;++x) {
           integratedAngleError[x]+=lib_fp_multiply(angleError[x],global.timesliver);
         
           // don't let the integrated error get too high (windup)
           lib_fp_constrain(&integratedAngleError[x],-INTEGRATEDANGLEERRORLIMIT,INTEGRATEDANGLEERRORLIMIT);
         
           // do the attitude pid
           pidoutput[x]=lib_fp_multiply(angleError[x],usersettings.pid_pgain[x])-lib_fp_multiply(global.gyrorate[x],usersettings.pid_dgain[x])+(lib_fp_multiply(integratedAngleError[x],usersettings.pid_igain[x])>>4);
            
           // add gain scheduling.
           pidoutput[x]=lib_fp_multiply(gainschedulingmultiplier,pidoutput[x]);
       }

       lib_fp_constrain(&throttleOutput,0,FIXEDPOINTONE);

       // set the final motor outputs
       // if we aren't armed, or if we desire to have the motors stop,
       if (!global.armed
#if (MOTORS_STOP==YES)
           || (global.rxValues[THROTTLE_INDEX]<FPSTICKLOW && !(global.activeCheckboxItems & (CHECKBOX_MASK_FULLACRO | CHECKBOX_MASK_SEMIACRO)))
#endif
           ) {
           set_all_motor_outputs(MIN_MOTOR_OUTPUT);
       } else {
          // mix the outputs to create motor values

#if (AIRCRAFT_CONFIGURATION==QUADX)
          setmotoroutput(0,MOTOR_0_CHANNEL,throttleOutput-pidoutput[ROLL_INDEX]+pidoutput[PITCH_INDEX]-pidoutput[YAW_INDEX]);
          setmotoroutput(1,MOTOR_1_CHANNEL,throttleOutput-pidoutput[ROLL_INDEX]-pidoutput[PITCH_INDEX]+pidoutput[YAW_INDEX]);
          setmotoroutput(2,MOTOR_2_CHANNEL,throttleOutput+pidoutput[ROLL_INDEX]+pidoutput[PITCH_INDEX]+pidoutput[YAW_INDEX]);
          setmotoroutput(3,MOTOR_3_CHANNEL,throttleOutput+pidoutput[ROLL_INDEX]-pidoutput[PITCH_INDEX]-pidoutput[YAW_INDEX]);
#endif
      }
   }
      
    return 0;   /* never reached */
}

void calculate_timesliver() {
    // load global.timesliver with the amount of time that has passed since we last went through this loop
    // convert from microseconds to fixedpointnum seconds shifted by TIMESLIVEREXTRASHIFT
    // 4295L is (FIXEDPOINTONE<<FIXEDPOINTSHIFT)*.000001
    global.timesliver=(lib_timers_gettimermicrosecondsandreset(&timeslivertimer)*4295L)>>(FIXEDPOINTSHIFT-TIMESLIVEREXTRASHIFT);

    // don't allow big jumps in time because of something slowing the update loop down (should never happen anyway)
    if (global.timesliver>(FIXEDPOINTONEFIFTIETH<<TIMESLIVEREXTRASHIFT)) global.timesliver=FIXEDPOINTONEFIFTIETH<<TIMESLIVEREXTRASHIFT;
}

void default_user_settings() {
    global.userSettingsFromEeprom=0; // this should get set to one if we read from eeprom
   
    // set default acro mode rotation rates
    usersettings.maxYawRate=200L<<FIXEDPOINTSHIFT; // degrees per second
    usersettings.maxPitchAndRollRate=400L<<FIXEDPOINTSHIFT; // degrees per second

    // set default PID settings
    for (int x=0;x<3;++x) {
        usersettings.pid_pgain[x]=15L<<3; // 1.5 on configurator
        usersettings.pid_igain[x]=8L;     // .008 on configurator
        usersettings.pid_dgain[x]=8L<<2;     // 8 on configurator
    }

    usersettings.pid_pgain[YAW_INDEX]=30L<<3; // 3 on configurator
   
    for (int x=3;x<NUM_PID_ITEMS;++x) {
        usersettings.pid_pgain[x]=0;
        usersettings.pid_igain[x]=0;
        usersettings.pid_dgain[x]=0;
    }
   
    usersettings.pid_pgain[ALTITUDE_INDEX]=27L<<7; // 2.7 on configurator
    usersettings.pid_dgain[ALTITUDE_INDEX]=6L<<9;     // 6 on configurator

    usersettings.pid_pgain[NAVIGATION_INDEX]=25L<<11; // 2.5 on configurator
    usersettings.pid_dgain[NAVIGATION_INDEX]=188L<<8;     // .188 on configurator
   
    // set default configuration checkbox settings.
    for (int x=0;x<NUM_POSSIBLE_CHECKBOXES;++x) {
        usersettings.checkboxConfiguration[x]=0;
    }

    //usersettings.checkboxConfiguration[CHECKBOX_ARM]=CHECKBOX_MASK_AUX1HIGH;
    usersettings.checkboxConfiguration[CHECKBOX_HIGHANGLE]=CHECKBOX_MASK_AUX1LOW;
    usersettings.checkboxConfiguration[CHECKBOX_SEMIACRO]=CHECKBOX_MASK_AUX1HIGH;
    usersettings.checkboxConfiguration[CHECKBOX_HIGHRATES]=CHECKBOX_MASK_AUX1HIGH;
   
    // reset the calibration settings
    for (int x=0;x<3;++x) {
        usersettings.compassZeroOffset[x]=0;
        usersettings.compassCalibrationMultiplier[x]=1L<<FIXEDPOINTSHIFT;
        usersettings.gyrocalibration[x]=0;
        usersettings.acccalibration[x]=0;
    }
}
