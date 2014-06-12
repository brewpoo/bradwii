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

#include "pilotcontrol.h"
#include "bradwii.h"
#include "vectors.h"
#include "lib_timers.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

// convert maximum tilt angles for level mode into fixed point
#define FP_LEVEL_MODE_MAX_TILT FIXEDPOINTCONSTANT(LEVEL_MODE_MAX_TILT)
#define FP_LEVEL_MODE_MAX_TILT_HIGH_ANGLE FIXEDPOINTCONSTANT(LEVEL_MODE_MAX_TILT_HIGH_ANGLE)
// convert high rate multiplier into fixed point
#define FP_HIGH_RATES_MULTIPLIER FIXEDPOINTCONSTANT(HIGH_RATES_MULTIPLIER)

// When the yaw stick is centered, allow compass hold.  This defines what centered is:
#define YAWCOMPASSRXDEADBAND FIXEDPOINTCONSTANT(.125) // 1/8 of the range

fixedpointnum filteredYawGyroRate=0;
fixedpointnum desiredCompassHeading;
fixedpointnum highyawrate;
fixedpointnum highPitchAndRollRate;

void reset_pilot_control() {
    // called when switching from navigation control to pilot control or when idling on the ground.
    // keeps us from accumulating yaw error that we can't correct.
    desiredCompassHeading=global.currentEstimatedEulerAttitude[YAW_INDEX];
   
    // calculate our max rotation rates based on usersettings
    highyawrate=lib_fp_multiply(usersettings.maxYawRate, FP_HIGH_RATES_MULTIPLIER);
    highPitchAndRollRate=lib_fp_multiply(usersettings.maxPitchAndRollRate, FP_HIGH_RATES_MULTIPLIER);
}

void get_angle_error_from_pilot_input(fixedpointnum *angleError) {
    // sets the ange errors for roll, pitch, and yaw based on where the pilot has the tx sticks.
    fixedpointnum rxRollValue;
    fixedpointnum rxPitchValue;
   
    // if in headfree mode, rotate the pilot's stick inputs by the angle that is the difference between where we are currently heading and where we were heading when we armed.
   if (global.activeCheckboxItems & CHECKBOX_MASK_HEADFREE) {
       fixedpointnum angledifference = global.currentEstimatedEulerAttitude[YAW_INDEX] - global.headingWhenArmed;

       fixedpointnum cosAngleDifference = lib_fp_cosine(angledifference);
       fixedpointnum sinAngleDifference = lib_fp_sine(angledifference);
       rxPitchValue = lib_fp_multiply(global.rxValues[PITCH_INDEX],cosAngleDifference) + lib_fp_multiply(global.rxValues[ROLL_INDEX],sinAngleDifference); // @FIXME
       rxRollValue =  lib_fp_multiply(global.rxValues[ROLL_INDEX],cosAngleDifference) - lib_fp_multiply(global.rxValues[PITCH_INDEX],sinAngleDifference);
   } else {
       rxPitchValue=global.rxValues[PITCH_INDEX];
       rxRollValue=global.rxValues[ROLL_INDEX];
   }
   
    // first, calculate level mode values
    // how far is our estimated current attitude from our desired attitude?
    // desired angle is rxvalue (-1 to 1) times LEVEL_MODE_MAX_TILT
    // First, figure out which max angle we are using depending on aux switch settings.
    fixedpointnum levelModeMaxAngle;
    if (global.activeCheckboxItems & CHECKBOX_MASK_HIGHANGLE) levelModeMaxAngle=FP_LEVEL_MODE_MAX_TILT_HIGH_ANGLE;
    else levelModeMaxAngle=FP_LEVEL_MODE_MAX_TILT;

    // the angle error is how much our current angles differ from our desired angles.
    fixedpointnum levelModeRollAngleError=lib_fp_multiply(rxRollValue,levelModeMaxAngle)-global.currentEstimatedEulerAttitude[ROLL_INDEX];
    fixedpointnum levelModePitchAngleError=lib_fp_multiply(rxPitchValue,levelModeMaxAngle)-global.currentEstimatedEulerAttitude[PITCH_INDEX];
   
    // In acro mode, we want the rotation rate to be proportional to the pilot's stick movement.  The desired rotation rate is
    // the stick movement * a multiplier.
    // Fill angleError with acro values.  If we are currently rotating at rate X and
    // we want to be rotating at rate Y, then our angle should be off by (Y-X)*timesliver.  In theory, we should probably accumulate
    // this error over time, but our I term will do that anyway and we are talking about rates, which don't need to be perfect.
    // First figure out what max rotation rates we are using depending on our aux switch settings.
    fixedpointnum maxYawRate;
    fixedpointnum maxPitchAndRollRate;
    if (global.activeCheckboxItems & CHECKBOX_MASK_HIGHRATES) {
        maxYawRate=highyawrate;
        maxPitchAndRollRate=highPitchAndRollRate;
    } else {
        maxYawRate=usersettings.maxYawRate;
        maxPitchAndRollRate=usersettings.maxPitchAndRollRate;
    }
      
    angleError[ROLL_INDEX]=lib_fp_multiply(lib_fp_multiply(rxRollValue,maxPitchAndRollRate)-global.gyrorate[ROLL_INDEX],global.timesliver);
    angleError[PITCH_INDEX]=lib_fp_multiply(lib_fp_multiply(rxPitchValue,maxPitchAndRollRate)-global.gyrorate[PITCH_INDEX],global.timesliver);

    // put a low pass filter on the yaw gyro.  If we don't do this, things can get jittery.
    lib_fp_lowpassfilter(&filteredYawGyroRate, global.gyrorate[YAW_INDEX], global.timesliver>>(TIMESLIVEREXTRASHIFT-3), FIXEDPOINTONEOVERONESIXTYITH, 3);

    // Calculate our yaw angle error
    angleError[YAW_INDEX]=lib_fp_multiply(lib_fp_multiply(global.rxValues[YAW_INDEX],maxYawRate)-filteredYawGyroRate,global.timesliver);

    // handle compass control
    if (global.activeCheckboxItems & CHECKBOX_MASK_COMPASS) {
        if (!(global.previousActiveCheckboxItems & CHECKBOX_MASK_COMPASS)) {
            // we just switched into compass mode
            // reset the angle error to zero so that we don't yaw because of the switch
            desiredCompassHeading=global.currentEstimatedEulerAttitude[YAW_INDEX];
        }
      
        // the compass holds only when right side up with throttle up (not sitting on the ground)
        // and the yaw stick is centered.  If it is centered, override the pilot's input
        if (global.estimatedDownVector[Z_INDEX]>0 && global.rxValues[THROTTLE_INDEX]>FPSTICKLOW && global.rxValues[YAW_INDEX]>-YAWCOMPASSRXDEADBAND && global.rxValues[YAW_INDEX]<YAWCOMPASSRXDEADBAND) {
            angleError[YAW_INDEX]=desiredCompassHeading-global.currentEstimatedEulerAttitude[YAW_INDEX];
            lib_fp_constrain180(&angleError[YAW_INDEX]);
        } else {
            // the pilot is controlling yaw, so update the desired heading
            desiredCompassHeading=global.currentEstimatedEulerAttitude[YAW_INDEX];
        }
    }
   
    // figure out how much to use acro mode and how much to use level mode.
    fixedpointnum acroModeFraction; // these two values should total FIXEDPOINTONE.  They are the weights applied to acro and level mode control
    fixedpointnum levelModeFraction;

    if (global.activeCheckboxItems & CHECKBOX_MASK_FULLACRO) {
        // acro mode
        acroModeFraction=FIXEDPOINTONE;
        levelModeFraction=0;
    } else if (!(global.activeCheckboxItems & CHECKBOX_MASK_SEMIACRO)) {
        // level mode
        acroModeFraction=0;
        levelModeFraction=FIXEDPOINTONE;
    } else {
        // semi acro mode
        // figure out how much the most moved stick is from centered
        fixedpointnum maxStickThrow;
   
        if (rxRollValue<0)
            maxStickThrow=-rxRollValue;
        else
            maxStickThrow=rxRollValue;
         
        if (rxPitchValue<0) {
            if (-rxPitchValue>maxStickThrow) maxStickThrow=-rxPitchValue;
        } else
            if (rxPitchValue>maxStickThrow) maxStickThrow=rxPitchValue;
         
   
        // if the aircraft is tipped more than 90 degrees, use full acro mode so we don't run into
        // euler issues when inverted.  This also allows us to pause while up-side-down if we want to.
        if (global.estimatedDownVector[Z_INDEX]<0) {
            acroModeFraction=FIXEDPOINTONE;
            levelModeFraction=0;
        } else {
            acroModeFraction=maxStickThrow;
      
            if (acroModeFraction>FIXEDPOINTONE) acroModeFraction=FIXEDPOINTONE;
            levelModeFraction=FIXEDPOINTONE-acroModeFraction;
        }
    }
      
    // combine level and acro modes
    angleError[ROLL_INDEX]=lib_fp_multiply(angleError[ROLL_INDEX], acroModeFraction)+lib_fp_multiply(levelModeRollAngleError, levelModeFraction);
    angleError[PITCH_INDEX]=lib_fp_multiply(angleError[PITCH_INDEX], acroModeFraction)+lib_fp_multiply(levelModePitchAngleError, levelModeFraction);

//if (1) // auto banking (experimental)
//   {
//   static fixedpointnum accrollangle=0;
//   
//   // calculate the current roll angle
//   
//   fixedpointnum newrollangle=  lib_fp_atan2(global.correctedVectorGs[X_INDEX] , global.correctedVectorGs[Z_INDEX]);
//
//   lib_fp_lowpassfilter(&accrollangle,newrollangle,global.timesliver,FIXEDPOINTCONSTANT(8),TIMESLIVEREXTRASHIFT);
//   angleError[ROLL_INDEX]-=accrollangle;
//global.debugValue[0]=newrollangle>>FIXEDPOINTSHIFT;
//global.debugValue[1]=accrollangle>>FIXEDPOINTSHIFT;
//   }
}
