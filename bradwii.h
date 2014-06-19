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

#include "lib_fp.h"
#include "rx.h"
#include "defs.h"
#include "checkboxes.h"
#include "vectors.h"

#define ROLL_INDEX 0
#define PITCH_INDEX 1
#define YAW_INDEX 2
#define THROTTLE_INDEX 3
#define AUX1_INDEX 4
#define AUX2_INDEX 5
#define AUX3_INDEX 6
#define AUX4_INDEX 7

#define ALTITUDE_INDEX 3 // pid gain index
#define NAVIGATION_INDEX 6 // pid gian index

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

#define NUM_PID_ITEMS 10

#define NAVIGATION_MODE_OFF 0
#define NAVIGATION_MODE_POSITION_HOLD 1
#define NAVIGATION_MODE_RETURN_TO_HOME 2
#define NAVIGATION_MODE_AUTOPILOT 3

typedef struct {
    unsigned char numSatelites;             // How many satelites do we currently see?
    fixedpointnum currentLatitude;          // The current GPS latitude
    fixedpointnum currentLongitude;         // The current GPS longitude
    fixedpointnum currentAltitude;          // The current GPS altitude
    fixedpointnum currentSpeed;             // The current GPS speed
} gpsDataStruct;

typedef struct {
    unsigned char armed;                    // A flag indicating that the aircraft is armed
    unsigned char stable;                   // Set to 1 when our gravity vector is close to unit length
    unsigned char altitudeHold;             // Altitude hold is engaged (how is this different from a navigation mode)
    unsigned char calibratingCompass;       // Flag indicating compass calibration is running
    unsigned char calibratingAccAndGyro;    // Flag indicating acc/gyro calibration is runnig
    unsigned char navigationMode;           // See navigation.h
} stateStruct;

typedef struct {
    fixedpointnum altitude;
    fixedpointnum latitude;
    fixedpointnum longitude;
} locationStruct;

typedef struct {
    fixedpointnum heading;          // the heading we were pointing when arming was established
    locationStruct location;        // the location we were in when armed
} homeStruct;

typedef struct {
    locationStruct location;        // 3D location of waypoint
    unsigned char waypointType;     // Type of waypoint: fly-over, hold, home
    fixedpointnum targetVelocity;   // Velocity of fly-over (units?)
    fixedpointnum holdDuration;     // Length of time to hold position (seconds)
} waypointStruct;

// put all of the global variables into one structure to make them easy to find
typedef struct {
    stateStruct state;                              // Current state
    homeStruct home;                                // Initial state (at arming)
    gpsDataStruct gps;                              // GPS state
    unsigned char userSettingsFromEeprom;           // set to 1 if user settings were read from eeprom
    fixedpointnum baroRawAltitude;                  // Current altitude read from barometer, in meters (approximately)
    fixedpointnum debugValue[4];                    // for display in the multiwii config program. Use for debugging.
    fixedpointnum timesliver;                       // The time in seconds (shifted TIMESLIVEREXTRASHIFT) since the last iteration of the main loop
    fixedpointnum gyrorate[3];                      // Corrected gyro rates in degrees per second
    fixedpointnum correctedVectorGs[3];             // Corrected accelerometer vector, in G's
    fixedpointnum altitude;                         // A filtered version of the baromemter's altitude
    fixedpointnum altitudeVelocity;                 // The rate of change of the altitude
    fixedpointnum altitudeHoldDesiredAltitude;      // The target altitude to hold
    fixedpointnum integratedAltitudeError;          // for pid control
    fixedpointnum integratedAngleError[3];          
    fixedpointnum estimatedDownVector[3];           // A unit vector (approximately) poining in the direction we think down is relative to the aircraft
    fixedpointnum estimatedWestVector[3];           // A unit vector (approximately) poining in the direction we think west is relative to the aircraft
    fixedpointnum currentEstimatedEulerAttitude[3]; // Euler Angles in degrees of how much we think the aircraft is Rolled, Pitched, and Yawed (from North)
    fixedpointnum rxValues[RX_NUM_CHANNELS];        // The values of the RX inputs, ranging from -1.0 to 1.0
    fixedpointnum compassVector[3];                 // A unit vector (approximately) poining in the direction our 3d compass is pointing
    unsigned int motorOutputValue[NUM_MOTORS];      // Output values to send to our motors, from 1000 to 2000
    unsigned int motor[NUM_MOTORS];                 // Values computed from the mix for motor output
    unsigned int servoOutputValue[NUM_SERVOS];      // Output values to send to our motors, from 1000 to 2000
    unsigned int servo[NUM_SERVOS];                 // Values computed from the mix for servo output
    unsigned long activeCheckboxItems;              // Bits for each checkbox item to show which are currently active
    unsigned long previousActiveCheckboxItems;      // The previous state of these bits so we can tell when they turn on and off
    fixedpointnum navigationDistance;               // The distance to to the navigation destination in meters (I think)
    fixedpointnum navigationBearing;                // The bearing from the last waypoint to the next one
    unsigned long failsafeTimer;                    // Timer for determining if we lose radio contact
} globalstruct;

typedef struct {
    unsigned int min;           // minimum value, must be more than 1020 with the current implementation
    unsigned int max;           // maximum value, must be less than 2000 with the current implementation
    unsigned int middle;        // default should be 1500
    unsigned char rate;         // range [-100;+100] ; can be used to adjust a rate 0-100% and a direction
    unsigned char direction;    // direction only +1 or -1
} servoStruct;

// put all of the user adjustable settings in one structure to make it easy to read and write to eeprom.
// We can add to the structure, but we shouldn't re-arrange the items to insure backward compatibility.
typedef struct {
    fixedpointnum maxYawRate;                               // maximum yaw rate (by pilot input) in degrees/sec
    fixedpointnum maxPitchAndRollRate;                      // maximum pitch and roll rate (by pilot input) in degrees/sec

    fixedpointnum pid_pgain[NUM_PID_ITEMS];                 // The various PID p gains
    fixedpointnum pid_igain[NUM_PID_ITEMS];                 // The various PID i gains
    fixedpointnum pid_dgain[NUM_PID_ITEMS];                 // The various PID d gains
    
    unsigned int checkboxConfiguration[NUM_POSSIBLE_CHECKBOXES]; // Bits that describe how the checkboxes are configured
    
    fixedpointnum gyroCalibration[3];                       // Offsets used to calibrate the gyro
    fixedpointnum accCalibration[3];                        // Offsets used to calibrate the accelerometer
    
    fixedpointnum compassCalibrationMultiplier[3];          // Multipliers used to calibrate the compass
    int compassZeroOffset[3];                               // Offsets used to calibrate the compass
    
    servoStruct servo[NUM_SERVOS];                          // Servo settings
    
    waypointStruct waypoints[NUM_WAYPOINTS];                // Waypoint storage
} settingsstruct;

void default_user_settings();
void calculate_timesliver();
