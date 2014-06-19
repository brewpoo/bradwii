/*
 Copyright 2014 Jon Lochner
 
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

#include "config.h"
#include "bradwii.h"
#include "autopilot.h"
#include "navigation.h"

#include "lib_fp.h"

extern globalstruct global;
extern settingsstruct settings;

#ifndef NO_AUTOPILOT

#define WAYPOINT_LAT settings.waypoints[destinationWaypoint].location.latitude
#define WAYPOINT_LON settings.waypoints[destinationWaypoint].location.longitude

#define FP_NEAR_WAYPOINT_DISTANCE FIXEDPOINTCONSTANT(AUTOPILOT_NEAR_WAYPOINT_RADIUS)

unsigned char traversing;
unsigned int currentWaypoint;
unsigned int destinationWaypoint;

void autopilot_init() {
    // reset the state of the autopilot to the beginning
    global.state.navigationMode=NAVIGATION_MODE_AUTOPILOT;
    destinationWaypoint=0;
    traversing=0;
}

bool anymore_waypoints() {
    // determine if there are any waypoints left to visit
    if (destinationWaypoint>NUM_WAYPOINTS) {
        return false;
    }
    if (!(settings.waypoints[destinationWaypoint].waypointType>WAYPOINT_TYPE_NONE)) {
        return false;
    }
    return true;
}

void navigate_to_next_waypoint() {
    // update the target location/altitude
    waypointStruct thisWaypoint = settings.waypoints[++destinationWaypoint];
    navigation_set_destination(thisWaypoint.location.latitude,thisWaypoint.location.longitude);
    global.altitudeHoldDesiredAltitude=thisWaypoint.location.altitude;
    global.state.altitudeHold = 1;
    traversing=1;
}

fixedpointnum distance_to_waypoint() {
    // if we are *near* the location of the waypoint
    fixedpointnum bearing;
    return navigation_get_distance_and_bearing(WAYPOINT_LAT,WAYPOINT_LON,global.gps.currentLatitude,global.gps.currentLongitude,&bearing);
}

bool reached_waypoint() {
    if (lib_fp_abs(distance_to_waypoint())<FP_NEAR_WAYPOINT_DISTANCE) {
        return true;
    }
    return false;
}

void autopilot_reset() {
    global.state.navigationMode=NAVIGATION_MODE_OFF;
    destinationWaypoint = 0;
    traversing = 0;
}

void autopilot(unsigned char autopilotState) {
    switch (autopilotState) {
        case AUTOPILOT_STARTING: {
            autopilot_init();
            break;
        }
            
        case AUTOPILOT_RUNNING: {
            if (reached_waypoint()) {
                if (anymore_waypoints()) {
                    navigate_to_next_waypoint();
                } else {
                    autopilot_reset();
                }
            }
            break;
        }
            
        case AUTOPILOT_STOPPING: {
            autopilot_reset();
            break;
        }
            
        default:
            break;
    }
}

#endif