/*
 Copyright 2014 Brad Quick, Jon Lochner
 
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

#include "uncrashable.h"
#include "config.h"
#include "bradwii.h"
#include "navigation.h"

#ifndef NO_UNCRASHABLE

extern globalstruct global;
extern usersettingsstruct usersettings;

#if (GPS_TYPE!=NO_GPS)
// keep a flag that tells us whether uncrashability is doing gps navigation or not
static unsigned char doingUncrashableNavigationFlag;
#endif
// we need a place to remember what the altitude was when uncrashability mode was turned on
static fixedpointnum uncrashabilityMinimumAltitude;
static fixedpointnum uncrasabilityDesiredAltitude;
static unsigned char doingUncrashableAltitudeHold=0;

void uncrashable(unsigned char gotNewGpsReading, fixedpointnum *angleError, fixedpointnum *throttleOutput) {
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
            global.altitudeHoldDesiredAltitude=uncrashabilityMinimumAltitude+FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET;
            global.integratedAltitudeError=0;
            global.state.altitudeHold=1;
        } else if (projectedAltitude<uncrashabilityMinimumAltitude) {
            // We are about to get below our minimum crashability altitude
            if (doingUncrashableAltitudeHold==0) { // if we just entered uncrashability, set our desired altitude to the current altitude
                uncrasabilityDesiredAltitude=global.altitude;
                global.integratedAltitudeError=0;
                doingUncrashableAltitudeHold=1;
            }
            
            // don't apply throttle until we are almost level
            if (global.estimatedDownVector[Z_INDEX]>FIXEDPOINTCONSTANT(.4)) {
                global.altitudeHoldDesiredAltitude=uncrasabilityDesiredAltitude;
                global.state.altitudeHold=1;
            } else throttleOutput=0;
                // we are trying to rotate to level, kill the throttle until we get there
                
                // make sure we are level!  Don't let the pilot command more than UNCRASHABLERECOVERYANGLE
                lib_fp_constrain(&angleError[ROLL_INDEX],-UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[ROLL_INDEX],UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[ROLL_INDEX]);
                lib_fp_constrain(&angleError[PITCH_INDEX],-UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[PITCH_INDEX],UNCRASHABLERECOVERYANGLE-global.currentEstimatedEulerAttitude[PITCH_INDEX]);
                } else doingUncrashableAltitudeHold=0;
                    
    #if (GPS_TYPE!=NO_GPS)
                    // Next, check to see if our GPS says we are out of bounds
                    // are we out of bounds?
                    fixedpointnum bearingFromHome;
        fixedpointnum distanceFromHome=navigation_get_distance_and_bearing(global.gps.currentLatitude,global.gps.currentLongitude,global.home.latitude,global.home.longitude,&bearingFromHome);
        
        if (distanceFromHome>FPUNCRASHABLE_RADIUS) { // we are outside the allowable area, navigate back toward home
            if (!doingUncrashableNavigationFlag) { // we just started navigating, so we have to set the destination
                navigation_set_destination(global.home.latitude,global.home.longitude);
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
}

#endif

