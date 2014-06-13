/* 
Copyright 2013 Brad Quick

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

#include <avr/io.h>

// library headers
#include "lib_timers.h"
#include "lib_fp.h" 

// project file headers
#include "bradwii.h"
#include "gyro.h" 
#include "accelerometer.h"
#include "baro.h"
#include "imu.h"
#include "compass.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

//fixedpointnum estimated_g_vector[3]={0,0,FIXEDPOINTONE}; // start pointing down
fixedpointnum estimated_compass_vector[3]={FIXEDPOINTONE,0,0}; // start pointing north

#define MAXACCMAGNITUDESQUARED FIXEDPOINTCONSTANT(1.1) // don't use acc to update attitude if under too many G's
#define MINACCMAGNITUDESQUARED FIXEDPOINTCONSTANT(0.9)

// convert MAG_DECLINIATION_DEGREES to fixed point
#define FP_MAG_DECLINATION_DEGREES FIXEDPOINTCONSTANT(MAG_DECLINATION_DEGREES)

// The following two rates can be defined in config.h to adjust imu performance.
// Ideally, we want to run mostly on the gyro since the gyro gives us instant feedback, but the gyro
// is integrated over time, which means that error continually accumulates and you will get drift.  The
// accelerometer is averaged in using a complimentary filter to gently remind us of which way gravity is
// pulling.  The acc value can be unreliable in the short term due to acceleration of the aircraft, but
// it works well over the long run.
// ACC_COMPLIMENTARY_FILTER_TIME_PERIOD adjusts how much the acc affects our positioning.

// Use the config program and look at the graphical representation of roll.  Rotate just the control board
// VERY quickly with a snap of the wrist so that the gyros can't keep up. Hold it there and
// you will see it creep slowly back to the proper angles.  The slow creep is caused by the acc complimentary
// filter.  By adjusting the ACC_COMPLIMENTARY_FILTER_TIME_PERIOD, you can make it creep
// more quickly or more slowly.  We want as slow a creep as is necessary. A larger value makes it creep more
// slowly.

#ifndef ACC_COMPLIMENTARY_FILTER_TIME_PERIOD
   #define ACC_COMPLIMENTARY_FILTER_TIME_PERIOD 2.0 // seconds
#endif

#define ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD FIXEDPOINTCONSTANT(1.0/ACC_COMPLIMENTARY_FILTER_TIME_PERIOD)

//fixedpointnum ; // convert from degrees to radians and include fudge factor
fixedpointnum baroTimeInterval=0;  // accumulated time between barometer reads
fixedpointnum compassTimeInterval=0; // accumulated time between barometer reads
fixedpointnum lastBaroRawAltitude; // remember our last reading so we can calculate altitude velocity

// read the acc and gyro a bunch of times and get an average of how far off they are.
// assumes the aircraft is sitting level and still.
void calibrate_gyro_and_accelerometer() {
    global.calibratingAccAndGyro = 1;
    for (int x=0;x<3;++x) {
        usersettings.gyrocalibration[x]=0;
        usersettings.acccalibration[x]=0;
    }

    fixedpointnum totaltime=0;
   
   // calibrate the gyro and acc
   while (totaltime<4L<<(FIXEDPOINTSHIFT+TIMESLIVEREXTRASHIFT)) {
       // 4 seconds
       read_gyro();
       read_acc();
       global.correctedVectorGs[Z_INDEX]-=FIXEDPOINTONE; // vertical vector should be at 1 G

       calculate_timesliver();
       totaltime+=global.timesliver;
      
       for (int x=0;x<3;++x) {
           lib_fp_lowpassfilter(&usersettings.gyrocalibration[x],-global.gyrorate[x],global.timesliver,FIXEDPOINTONEOVERONE,TIMESLIVEREXTRASHIFT);
           lib_fp_lowpassfilter(&usersettings.acccalibration[x],-global.correctedVectorGs[x],global.timesliver,FIXEDPOINTONEOVERONE,TIMESLIVEREXTRASHIFT);
       }
   }
    global.calibratingAccAndGyro = 0;
}

void init_imu() {
    // calibrate every time if we dont load any data from eeprom
    if (global.userSettingsFromEeprom==0) calibrate_gyro_and_accelerometer();

    global.estimatedDownVector[X_INDEX]=0;
    global.estimatedDownVector[Y_INDEX]=0;
    global.estimatedDownVector[Z_INDEX]=FIXEDPOINTONE;
   
    global.estimatedWestVector[X_INDEX]=FIXEDPOINTONE;
    global.estimatedWestVector[Y_INDEX]=0;
    global.estimatedWestVector[Z_INDEX]=0;
   
    lastBaroRawAltitude=global.altitude=global.baroRawAltitude;

    global.altitudeVelocity=0;
}

//fixedpointnum totalrate[3]={0};
//fixedpointnum timesincezerocrossing[3]={0};
//char gyropositive[3]={0};

void imu_calculate_estimated_attitude() {
    read_gyro();
    read_acc();

    // correct the gyro and acc readings to remove error
    for (int x=0;x<3;++x) {
        global.gyrorate[x]=global.gyrorate[x]+usersettings.gyrocalibration[x];
        global.correctedVectorGs[x]=global.correctedVectorGs[x]+usersettings.acccalibration[x];
    }

    // calculate how many degrees we have rotated around each axis.  Keep in mind that timesliver is
    // shifted TIMESLIVEREXTRASHIFT bits to the left, so our delta angles will be as well.  This is
    // good because they are generally very small angles;
   
    // create a multiplier that will include timesliver and a conversion from degrees to radians
    // we need radians for small angle approximation
    fixedpointnum multiplier=lib_fp_multiply(global.timesliver,FIXEDPOINTPIOVER180);

    fixedpointnum rolldeltaangle=lib_fp_multiply(global.gyrorate[ROLL_INDEX],multiplier);
    fixedpointnum pitchdeltaangle=lib_fp_multiply(global.gyrorate[PITCH_INDEX],multiplier);
    fixedpointnum yawdeltaangle=lib_fp_multiply(global.gyrorate[YAW_INDEX],multiplier);

    rotate_vector_with_small_angles(global.estimatedDownVector,rolldeltaangle,pitchdeltaangle,yawdeltaangle);
    rotate_vector_with_small_angles(global.estimatedWestVector,rolldeltaangle,pitchdeltaangle,yawdeltaangle);

    // if the accellerometer's gravity vector is close to one G, use a complimentary filter
    // to gently adjust our estimated g vector so that it stays in line with the real one.
    // If the magnitude of the vector is not near one G, then it will be difficult to determine
    // which way is down, so we just skip it.
    fixedpointnum accmagnitudesquared=lib_fp_multiply(global.correctedVectorGs[X_INDEX],global.correctedVectorGs[X_INDEX])+lib_fp_multiply(global.correctedVectorGs[Y_INDEX],global.correctedVectorGs[Y_INDEX])+lib_fp_multiply(global.correctedVectorGs[Z_INDEX],global.correctedVectorGs[Z_INDEX]);

    if (accmagnitudesquared>MINACCMAGNITUDESQUARED && accmagnitudesquared<MAXACCMAGNITUDESQUARED) {
        global.stable=1;
        for (int x=0;x<3;++x) {
            lib_fp_lowpassfilter(&global.estimatedDownVector[x], global.correctedVectorGs[x],global.timesliver, ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD,TIMESLIVEREXTRASHIFT);
        }
    } else global.stable=0;

    compassTimeInterval+=global.timesliver;

#if (COMPASS_TYPE!=NO_COMPASS)
    int gotNewCompassReading=read_compass();
      
    if (gotNewCompassReading) {
        // use the compass to correct the yaw in our estimated attitude.
        // the compass vector points somewhat north, but it also points down more than north where I live, so we can't
        // get the yaw directly from the compass vector.  Instead, we have to take a cross product of
        // the gravity vector and the compass vector, which should point west
        fixedpointnum westVector[3];
      
        vector_cross_product(global.compassVector,global.estimatedDownVector,westVector);

        // use the actual compass reading to slowly adjust our estimated west vector
        for (int x=0;x<3;++x) {
            lib_fp_lowpassfilter(&global.estimatedWestVector[x], westVector[x],compassTimeInterval>>TIMESLIVEREXTRASHIFT, ONE_OVER_ACC_COMPLIMENTARY_FILTER_TIME_PERIOD,0);
        }
        compassTimeInterval=0;
    }
#else
    if (compassTimeInterval>(6553L<<TIMESLIVEREXTRASHIFT)) {
        // 10 hz
        // we aren't using the comopass
        // we need to make sure the west vector stays around unit length and stays perpendicular to the down vector
        // first make it perpendicular by crossing it with the down vector and then back again
        fixedpointnum vector[3];
      
        vector_cross_product(global.estimatedWestVector, global.estimatedDownVector,vector);
        vector_cross_product(global.estimatedDownVector,vector, global.estimatedWestVector);

        normalize_vector(global.estimatedWestVector);
      
        compassTimeInterval=0;
    }
#endif

#if (BAROMETER_TYPE!=NO_BAROMETER)
    baroTimeInterval+=global.timesliver;
   
    // Integrate the accelerometer to determine the altitude velocity
    // Integrate again to determine position
    //normalize_vector(global.estimatedDownVector);

   fixedpointnum verticalacceleration=lib_fp_multiply(vector_dot_product(global.correctedVectorGs, global.estimatedDownVector)-FIXEDPOINTONE,FIXEDPOINTCONSTANT(9.8));
   global.altitudeVelocity+=(lib_fp_multiply(verticalacceleration>>TIMESLIVEREXTRASHIFT, global.timesliver));
   global.altitude+=(lib_fp_multiply(global.altitudeVelocity>>TIMESLIVEREXTRASHIFT, global.timesliver));

    if (read_baro()) {
        // we got a new baro reading
        fixedpointnum baroaltitudechange=global.baroRawAltitude-lastBaroRawAltitude;
      
        // filter out errant baro readings.  I don't know why I need to do this, but every once in a while the baro
        // will give a reading of 3000 meters when it should read 150 meters.
        if (lib_fp_abs(baroaltitudechange)<FIXEDPOINTCONSTANT(500)) {
            // Use the baro reading to adjust the altitude over time (basically a complimentary filter)
            lib_fp_lowpassfilter(&global.altitude, global.baroRawAltitude,baroTimeInterval>>TIMESLIVEREXTRASHIFT, FIXEDPOINTONEOVERONE,0);

            // Use the change in barometer readings to get an altitude velocity.  Use this to adjust the altitude velocity
            // over time (basically a complimentary filter).
            // We don't want to divide by the time interval to get velocity (divide is expensive) to then turn around and
            // multiply by the same time interval. So the following is the same as the lib_fp_lowpassfilter code
            // except we eliminate the multiply.
            fixedpointnum fraction=lib_fp_multiply(baroTimeInterval>>TIMESLIVEREXTRASHIFT,FIXEDPOINTONEOVERONEHALF);
            global.altitudeVelocity=(baroaltitudechange+lib_fp_multiply((FIXEDPOINTONE)-fraction,global.altitudeVelocity));
      
            lastBaroRawAltitude=global.baroRawAltitude;
            baroTimeInterval=0;
        }
    }
#endif   
      
    // convert our vectors to euler angles
    global.currentEstimatedEulerAttitude[ROLL_INDEX]  =  lib_fp_atan2(global.estimatedDownVector[X_INDEX] , global.estimatedDownVector[Z_INDEX]) ;
    if (lib_fp_abs(global.currentEstimatedEulerAttitude[ROLL_INDEX])>FIXEDPOINT45 && lib_fp_abs(global.currentEstimatedEulerAttitude[ROLL_INDEX])<FIXEDPOINT135) {
        global.currentEstimatedEulerAttitude[PITCH_INDEX] = lib_fp_atan2(global.estimatedDownVector[Y_INDEX] , lib_fp_abs(global.estimatedDownVector[X_INDEX]));
    } else {
        global.currentEstimatedEulerAttitude[PITCH_INDEX] = lib_fp_atan2(global.estimatedDownVector[Y_INDEX] , global.estimatedDownVector[Z_INDEX]);
    }

    fixedpointnum xvalue=global.estimatedWestVector[X_INDEX];
   
    if (global.estimatedDownVector[Z_INDEX]<1) xvalue=-xvalue;
      
    global.currentEstimatedEulerAttitude[YAW_INDEX] = lib_fp_atan2(global.estimatedWestVector[Y_INDEX],xvalue)+FP_MAG_DECLINATION_DEGREES;
    lib_fp_constrain180(&global.currentEstimatedEulerAttitude[YAW_INDEX]);
}
