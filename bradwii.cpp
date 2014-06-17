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
#include "aircraft.h"
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
#include "uncrashable.h"
#include "autotune.h"

globalstruct global; // global variables
settingsstruct settings; // user editable variables

// limit pid windup
#define INTEGRATED_ANGLE_ERROR_LIMIT FIXEDPOINTCONSTANT(5000) 

// timesliver is a very small slice of time (.002 seconds or so).  This small value doesn't take much advantage
// of the resolution of fixedpointnum, so we shift timesliver an extra TIMESLIVEREXTRASHIFT bits.
unsigned long timeslivertimer=0;

// It all starts here:
int main(void) {
    // start with default user settings in case there's nothing in eeprom
    default_user_settings();
    // try to load settings from eeprom
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

    // initialize state
    global.state.armed=0;
    global.state.calibratingCompass=0;
    global.state.calibratingAccAndGyro=0;
    global.state.navigationMode=NAVIGATION_MODE_OFF;
    global.failsafeTimer=lib_timers_starttimer();

    // run loop
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
          if (!global.state.armed) {
             if (global.activeCheckboxItems & CHECKBOX_MASK_ARM) {
                 global.state.armed=1;
                #if (GPS_TYPE!=NO_GPS)
                 navigation_set_home_to_current_location();
                #endif
                 global.home.heading=global.currentEstimatedEulerAttitude[YAW_INDEX];
                 global.home.altitude=global.baroRawAltitude;
             }
          } else if (!(global.activeCheckboxItems & CHECKBOX_MASK_ARM)) global.state.armed=0;
      }

      #if (GPS_TYPE!=NO_GPS)
      // turn on or off navigation when appropriate
      if (global.state.navigationMode==NAVIGATION_MODE_OFF) {
          if (global.activeCheckboxItems & CHECKBOX_MASK_RETURNTOHOME) { // return to home switch turned on
              navigation_set_destination(global.home.latitude,global.home.longitude);
              global.state.navigationMode=NAVIGATION_MODE_RETURN_TO_HOME;
          } else if (global.activeCheckboxItems & CHECKBOX_MASK_POSITIONHOLD) { // position hold turned on
              navigation_set_destination(global.gps.currentLatitude,global.gps.currentLongitude);
              global.state.navigationMode=NAVIGATION_MODE_POSITION_HOLD;
          }
      } else { // we are currently navigating
          // turn off navigation if desired
          if ((global.state.navigationMode==NAVIGATION_MODE_RETURN_TO_HOME && !(global.activeCheckboxItems & CHECKBOX_MASK_RETURNTOHOME)) || (global.state.navigationMode==NAVIGATION_MODE_POSITION_HOLD && !(global.activeCheckboxItems & CHECKBOX_MASK_POSITIONHOLD))) {
              global.state.navigationMode=NAVIGATION_MODE_OFF;
            
              // we will be turning control back over to the pilot.
              reset_pilot_control();
          }
      }
        #endif
      
       // read the receiver
       read_rx();
      
       // turn on the LED when we are stable and the gps has 5 satelites or more
      #if (GPS_TYPE==NO_GPS)
       lib_digitalio_setoutput(LED1_OUTPUT, (global.state.stable==0)==LED1_ON);
      #else
       lib_digitalio_setoutput(LED1_OUTPUT, (!(global.state.stable && global.gps.numSatelites>=5))==LED1_ON);
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
       if (global.state.navigationMode!=NAVIGATION_MODE_OFF) { // we are navigating
           navigation_set_angle_error(gotNewGpsReading,angleError);
       }
#endif

       if (global.rxValues[THROTTLE_INDEX]<FPSTICKLOW) {
           // We are probably on the ground. Don't accumnulate error when we can't correct it
           reset_pilot_control();
         
           // bleed off integrated error by averaging in a value of zero
           lib_fp_lowpassfilter(&global.integratedAngleError[ROLL_INDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
           lib_fp_lowpassfilter(&global.integratedAngleError[PITCH_INDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
           lib_fp_lowpassfilter(&global.integratedAngleError[YAW_INDEX],0L,global.timesliver>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEFOURTH,0);
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
       global.home.altitude=0;
      
       if (global.activeCheckboxItems & CHECKBOX_MASK_ALTHOLD) {
           global.state.altitudeHold=1;
           if (!(global.previousActiveCheckboxItems & CHECKBOX_MASK_ALTHOLD)) {
               // we just turned on alt hold.  Remember our current alt. as our target
               global.altitudeHoldDesiredAltitude=global.altitude;
               global.integratedAltitudeError=0;
           }
       }
       
#ifndef NO_UNCRASHABLE
        uncrashable(gotNewGpsReading,angleError,&throttleOutput);
#endif
        
#if (BAROMETER_TYPE!=NO_BAROMETER)
       // check for altitude hold and adjust the throttle output accordingly
       if (global.state.altitudeHold) {
           global.integratedAltitudeError+=lib_fp_multiply(global.altitudeHoldDesiredAltitude-global.altitude,global.timesliver);
           lib_fp_constrain(&global.integratedAltitudeError,-INTEGRATED_ANGLE_ERROR_LIMIT,INTEGRATED_ANGLE_ERROR_LIMIT); // don't let the integrated error get too high
         
           // do pid for the altitude hold and add it to the throttle output
           throttleOutput+=lib_fp_multiply(global.altitudeHoldDesiredAltitude-global.altitude,settings.pid_pgain[ALTITUDE_INDEX])-lib_fp_multiply(global.altitudeVelocity,settings.pid_dgain[ALTITUDE_INDEX])+lib_fp_multiply(global.integratedAltitudeError,settings.pid_igain[ALTITUDE_INDEX]);
       }
#endif
       if ((global.activeCheckboxItems & CHECKBOX_MASK_AUTOTHROTTLE) || global.state.altitudeHold) {
           // Auto Throttle Adjust - Increases the throttle when the aircraft is tilted so that the vertical
           // component of thrust remains constant.
           // The AUTOTHROTTLE_DEAD_AREA adjusts the value at which the throttle starts taking effect.  If this
           // value is too low, the aircraft will gain altitude when banked, if it's too low, it will lose
           // altitude when banked. Adjust to suit.
            #define AUTOTHROTTLE_DEAD_AREA FIXEDPOINTCONSTANT(.25)

           if (global.estimatedDownVector[Z_INDEX]>FIXEDPOINTCONSTANT(.3)) {
               // Divide the throttle by the throttleOutput by the z component of the down vector
               // This is probaly the slow way, but it's a way to do fixed point division
               fixedpointnum recriprocal=lib_fp_invsqrt(global.estimatedDownVector[Z_INDEX]);
               recriprocal=lib_fp_multiply(recriprocal,recriprocal);
         
               throttleOutput=lib_fp_multiply(throttleOutput-AUTOTHROTTLE_DEAD_AREA,recriprocal)+AUTOTHROTTLE_DEAD_AREA;
           }
       }

#ifndef NO_FAILSAFE
       // if we don't hear from the receiver for over a second, try to land safely
       if (lib_timers_gettimermicroseconds(global.failsafeTimer)>1000000L) {
           throttleOutput=FPFAILSAFEMOTOROUTPUT;

           // make sure we are level!
           angleError[ROLL_INDEX]=-global.currentEstimatedEulerAttitude[ROLL_INDEX];
           angleError[PITCH_INDEX]=-global.currentEstimatedEulerAttitude[PITCH_INDEX];
       }
#endif
        
       // calculate output values.  Output values will range from 0 to 1.0

       // calculate pid outputs based on our angleErrors as inputs
       fixedpointnum pidOutput[3];
      
       // Gain Scheduling essentialy modifies the gains depending on
       // throttle level. If GAIN_SCHEDULING_FACTOR is 1.0, it multiplies PID outputs by 1.5 when at full throttle,
       // 1.0 when at mid throttle, and .5 when at zero throttle.  This helps
       // eliminate the wobbles when decending at low throttle.
       fixedpointnum gainSchedulingMultiplier=lib_fp_multiply(throttleOutput-FIXEDPOINTCONSTANT(.5),FIXEDPOINTCONSTANT(GAIN_SCHEDULING_FACTOR))+FIXEDPOINTONE;
      
       for (int x=0;x<3;++x) {
           global.integratedAngleError[x]+=lib_fp_multiply(angleError[x],global.timesliver);
         
           // don't let the integrated error get too high (windup)
           lib_fp_constrain(&global.integratedAngleError[x],-INTEGRATED_ANGLE_ERROR_LIMIT,INTEGRATED_ANGLE_ERROR_LIMIT);
         
           // do the attitude pid
           pidOutput[x]=lib_fp_multiply(angleError[x],settings.pid_pgain[x])-lib_fp_multiply(global.gyrorate[x],settings.pid_dgain[x])+(lib_fp_multiply(global.integratedAngleError[x],settings.pid_igain[x])>>4);
            
           // add gain scheduling.
           pidOutput[x]=lib_fp_multiply(gainSchedulingMultiplier,pidOutput[x]);
       }

       lib_fp_constrain(&throttleOutput,0,FIXEDPOINTONE); // Keep throttle output between 0 and 1

       compute_mix(throttleOutput, pidOutput); // aircraft type dependent mixes
       
#if (NUM_SERVOS>0)
       // do not update servos during unarmed calibration of sensors which are sensitive to vibration
       if (global.state.armed || (!global.state.calibratingAccAndGyro)) write_servo_outputs();
#endif
       
       write_motor_outputs();
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
    settings.maxYawRate=200L<<FIXEDPOINTSHIFT; // degrees per second
    settings.maxPitchAndRollRate=400L<<FIXEDPOINTSHIFT; // degrees per second

    // set default PID settings
    for (int x=0;x<3;++x) {
        settings.pid_pgain[x]=15L<<3; // 1.5 on configurator
        settings.pid_igain[x]=8L;     // .008 on configurator
        settings.pid_dgain[x]=8L<<2;     // 8 on configurator
    }

    settings.pid_pgain[YAW_INDEX]=30L<<3; // 3 on configurator
   
    for (int x=3;x<NUM_PID_ITEMS;++x) {
        settings.pid_pgain[x]=0;
        settings.pid_igain[x]=0;
        settings.pid_dgain[x]=0;
    }
   
    settings.pid_pgain[ALTITUDE_INDEX]=27L<<7; // 2.7 on configurator
    settings.pid_dgain[ALTITUDE_INDEX]=6L<<9;     // 6 on configurator

    settings.pid_pgain[NAVIGATION_INDEX]=25L<<11; // 2.5 on configurator
    settings.pid_dgain[NAVIGATION_INDEX]=188L<<8;     // .188 on configurator
   
    // set default configuration checkbox settings.
    for (int x=0;x<NUM_POSSIBLE_CHECKBOXES;++x) {
        settings.checkboxConfiguration[x]=0;
    }

    //settings.checkboxConfiguration[CHECKBOX_ARM]=CHECKBOX_MASK_AUX1HIGH;
    settings.checkboxConfiguration[CHECKBOX_HIGHANGLE]=CHECKBOX_MASK_AUX1LOW;
    settings.checkboxConfiguration[CHECKBOX_SEMIACRO]=CHECKBOX_MASK_AUX1HIGH;
    settings.checkboxConfiguration[CHECKBOX_HIGHRATES]=CHECKBOX_MASK_AUX1HIGH;
   
    // reset the calibration settings
    for (int x=0;x<3;++x) {
        settings.compassZeroOffset[x]=0;
        settings.compassCalibrationMultiplier[x]=1L<<FIXEDPOINTSHIFT;
        settings.gyrocalibration[x]=0;
        settings.acccalibration[x]=0;
    }
    
#if NUM_SERVOS>0
    for (int x=0;x<NUM_SERVOS;x++) {
        settings.servo[x].min=1020;
        settings.servo[x].max=2000;
        settings.servo[x].middle=1500;
        settings.servo[x].rate=100;
        settings.servo[x].direction=1;
    }
#endif
    
}
