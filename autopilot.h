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

#define AUTOPILOT_OFF       0
#define AUTOPILOT_STARTING  1
#define AUTOPILOT_RUNNING   2
#define AUTOPILOT_STOPPING  3

#define WAYPOINT_TYPE_NONE          0
#define WAYPOINT_TYPE_FLYOVER       1
#define WAYPOINT_TYPE_LOITER        2
#define WAYPOINT_TYPE_RETURN_HOME   3

#define FP_AUTOPILOT_MAX_ANGLE FIXEDPOINTCONSTANT(15) // don't let the autopilot pitch or roll more than 15 degrees
#define FP_AUTOPILOT_NEAR_RADIUS FIXEDPOINTCONSTANT(AUTOPILOT_NEAR_WAYPOINT_RADIUS)
#define FP_AUTOPILOT_MAX_SPEED FIXEDPOINTCONSTANT(AUTOPILOT_MAX_SPEED)

#define NAV_WAYPOINT_FLYOVER    0
#define NAV_WAYPOINT_PAUSE      1

void autopilot(unsigned char autopilotState);

