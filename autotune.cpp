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

// The theory behind auto tuning is that we generate a step in the setpoint, then analyze how the system reacts.
// First, we set the I term to zero and work on only the P and D terms.
// The target angle alternates from negative to posive FPAUTOTUNETARGETANGLE degrees.  We set the target, then wait until
// we cross the zero angle, just to make sure our angular velocity is in the right direction, then we look for the peak
// which we recognize by observing the angular velocity switching direction.
// Our goal is to get zero overshoot.  By requiring zero overshoot, we can assume that any subsequent oscillations are
// caused by our D term, since at zero overshoot, the P term doesn't have any effect.  Once we see the first peak, we then
// start a timer and look for a second peak in the negative direction.  The difference between the first and second peak will
// tell us the effect the D term is having.  We will then try to keep this oscillation amplitude within a predeterrmined range.
//
// So, after we reach the target, then wait a little longer to record the peak oscillation, we have four choices:
// if (overshoot)
//      {
//      if (too much oscillation) decrease P
//      else increase D
//      }
//   else // undershoot
//      { 
//      if (too much oscillation) decrease D
//      else increase P
//      }


#include "config.h"
#include "bradwii.h"
#include "eeprom.h"
#include "autotune.h"

extern globalstruct global;
extern usersettingsstruct usersettings;

#ifndef NO_AUTOTUNE

#define FPAUTOTUNEMAXOSCILLATION FIXEDPOINTCONSTANT(AUTOTUNE_MAX_OSCILLATION)
#define FPAUTOTUNETARGETANGLE FIXEDPOINTCONSTANT(AUTOTUNE_TARGET_ANGLE)

unsigned char rising;
int autotuneIndex=ROLL_INDEX;
fixedpointnum autotuneTime;
fixedpointnum autotunePeak1;
fixedpointnum autotunePeak2;
fixedpointnum targetAngle=0;
fixedpointnum targetAngleAtPeak;
fixedpointnum currentPValueShifted;
fixedpointnum currentIValueShifted;
fixedpointnum currentDValueShifted;

char cyclecount=1;

void autotune(fixedpointnum *angleError,unsigned char startingOrStopping) {
    if (!global.armed) {
        // we aren't armed.  Don't do anything, but if autotuning is started and we have collected
        // autotuning data, save our settings to eeprom
        if (startingOrStopping==AUTOTUNESTARTING && targetAngle!=0) write_user_settings_to_eeprom();
        return;
    }
      
    if (startingOrStopping==AUTOTUNESTOPPING) {
        usersettings.pid_igain[autotuneIndex]=currentIValueShifted>>AUTOTUNESHIFT;
      
        // multiply by D multiplier.  The best D is usually a little higher than what the algroithm produces.
        usersettings.pid_dgain[autotuneIndex]=lib_fp_multiply(currentDValueShifted,FPAUTOTUNE_D_MULTIPLIER)>>AUTOTUNESHIFT;
      
        usersettings.pid_igain[YAW_INDEX]=usersettings.pid_igain[ROLL_INDEX];
        usersettings.pid_dgain[YAW_INDEX]=usersettings.pid_dgain[ROLL_INDEX];
        usersettings.pid_pgain[YAW_INDEX]=lib_fp_multiply(usersettings.pid_pgain[ROLL_INDEX],YAWGAINMULTIPLIER);

        autotuneIndex=!autotuneIndex; // alternate between roll and pitch
        return;
    }

    if (startingOrStopping==AUTOTUNESTARTING) {
        currentPValueShifted=usersettings.pid_pgain[autotuneIndex]<<AUTOTUNESHIFT;
        currentIValueShifted=usersettings.pid_igain[autotuneIndex]<<AUTOTUNESHIFT;
        
        // divide by D multiplier to get our working value.  We'll multiply by D multiplier when we are done.
        usersettings.pid_dgain[autotuneIndex]=lib_fp_multiply(usersettings.pid_dgain[autotuneIndex],FPONEOVERAUTOTUNE_D_MULTIPLIER);
        currentDValueShifted=usersettings.pid_dgain[autotuneIndex]<<AUTOTUNESHIFT;
      
        usersettings.pid_igain[autotuneIndex]=0;
        cyclecount=1;
        autotunePeak1=autotunePeak2=0;
        rising=0;
    } else {
        // we are autotuning.  Analyze our current data.
        fixedpointnum currentAngle;

        if (rising) {
            currentAngle=global.currentEstimatedEulerAttitude[autotuneIndex];
        } else {
            // convert the numbers so it looks as if we are working with a positive target
            targetAngle=-targetAngle;
            currentAngle=-global.currentEstimatedEulerAttitude[autotuneIndex];
        }
      
        if (autotunePeak2==0) {
            // we haven't seen the first peak yet
            // The peak will be when our angular velocity is negative.  To be sure we are in the right place,
            // we also check to make sure our angle position is greater than zero.
            if (currentAngle>autotunePeak1) {
                // we are still going up
                autotunePeak1=currentAngle;
                targetAngleAtPeak=targetAngle;
            } else if (autotunePeak1>0) {
                // we have changed direction.  We have seen the first peak.
                if (cyclecount==0) {
                    // we are checking the I value
                    // when checking the I value, we would like to overshoot the target position by half of the max oscillation.
                    if (currentAngle-targetAngle<(FPAUTOTUNEMAXOSCILLATION>>1)) {
                        currentIValueShifted=lib_fp_multiply(currentIValueShifted,AUTOTUNEINCREASEMULTIPLIER);
                    } else {
                        currentIValueShifted=lib_fp_multiply(currentIValueShifted,AUTOTUNEDECREASEMULTIPLIER);
                        if (currentIValueShifted<AUTOTUNEMINIMUMIVALUE) currentIValueShifted=AUTOTUNEMINIMUMIVALUE;
                    }

                    // go back to checking P and D
                    cyclecount=1;
                    rising=!rising;
                    usersettings.pid_igain[autotuneIndex]=0;
                    autotunePeak1=autotunePeak2=0;
                } else {
                    // we are checking P and D values
                    // get set up to look for the 2nd peak
                    autotunePeak2=currentAngle;
                    autotuneTime=0;
                }
            }
        } else {
            // we saw the first peak. looking for the second
            autotuneTime+=global.timesliver;
         
            if (currentAngle<autotunePeak2) autotunePeak2=currentAngle;
            
            fixedpointnum oscillationamplitude=autotunePeak1-autotunePeak2;
     
            // stop looking for the 2nd peak if we time out or if we change direction again after moving by more than half the maximum oscillation
            if (autotuneTime>AUTOTUNESETTLINGTIME || (oscillationamplitude>(FPAUTOTUNEMAXOSCILLATION>>1) && currentAngle>autotunePeak2)) {
                // analyze the data
                // Our goal is to have zero overshoot and to have AUTOTUNEMAXOSCILLATION amplitude
                if (autotunePeak1>targetAngleAtPeak) {
                    // overshoot
                    // by removing the if and else, we tend to push toward a higher gain solution in the long run
   //            if (oscillationamplitude>FPAUTOTUNEMAXOSCILLATION) // we have too much oscillation, so we can't increase D, so decrease P
                    currentPValueShifted=lib_fp_multiply(currentPValueShifted, AUTOTUNEDECREASEMULTIPLIER);
  //             else // we don't have too much oscillation, so we can increase D
                    currentDValueShifted=lib_fp_multiply(currentDValueShifted, AUTOTUNEINCREASEMULTIPLIER);
                } else {
                    // undershoot
                    if (oscillationamplitude>FPAUTOTUNEMAXOSCILLATION) { // we have too much oscillation, so we should lower D
                        currentDValueShifted=lib_fp_multiply(currentDValueShifted, AUTOTUNEDECREASEMULTIPLIER);
                    } else {
                        // we don't have too much oscillation, so we increase P
                        currentPValueShifted=lib_fp_multiply(currentPValueShifted, AUTOTUNEINCREASEMULTIPLIER);
                    }
                }
               
                usersettings.pid_pgain[autotuneIndex]=currentPValueShifted>>AUTOTUNESHIFT;
                usersettings.pid_dgain[autotuneIndex]=currentDValueShifted>>AUTOTUNESHIFT;
            
                // switch to the other direction and start a new cycle
                rising=!rising;
                autotunePeak1=autotunePeak2=0;
         
                if (++cyclecount==3) {
                    // switch to testing I value
                    cyclecount=0;
            
                    usersettings.pid_igain[autotuneIndex]=currentIValueShifted>>AUTOTUNESHIFT;
                }
            }
        }
    }
   
    if (rising) {
        targetAngle=/*global.rxValues[autotuneIndex]*20L+*/FPAUTOTUNETARGETANGLE;
    } else {
        targetAngle=/*global.rxValues[autotuneIndex]*20L*/-FPAUTOTUNETARGETANGLE;
    }
   
   // override the angle error for the axis we are tuning
    angleError[autotuneIndex]=targetAngle-global.currentEstimatedEulerAttitude[autotuneIndex];
}

#endif