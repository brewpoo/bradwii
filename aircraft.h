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

#include "options.h"
#include "config.h"
#include "lib_fp.h"

// set aircraft type dependant defines here
#if (AIRCRAFT_CONFIGURATION==QUADX)
    #define NUM_MOTORS 4
    #define QUADX_REAR_RIGHT_MOTOR 0
    #define MOTOR_0_CHANNEL D3_PWM
    #define MOTOR_0_PIN     D3_PIN

    #define QUADX_FRONT_RIGHT_MOTOR 1
    #define MOTOR_1_CHANNEL D5_PWM
    #define MOTOR_1_PIN     D5_PIN

    #define QUADX_REAR_LEFT_MOTOR 2
    #define MOTOR_2_CHANNEL D6_PWM
    #define MOTOR_2_PIN     D6_PIN

    #define QUADX_FRONT_LEFT_MOTOR 3
    #define MOTOR_3_CHANNEL D2_PWM
    #define MOTOR_3_PIN     D2_PIN

    #define NUM_SERVOS 0

#elif (AIRCRAFT_CONFIGURATION==TRI)
    #define NUM_MOTORS 3
    #define TRI_REAR_MOTOR 0
    #define MOTOR_0_CHANNEL D3_PWM
    #define MOTOR_0_PIN     D3_PIN

    #define TRI_LEFT_MOTOR 1
    #define MOTOR_1_CHANNEL D6_PWM
    #define MOTOR_1_PIN     D6_PIN

    #define TRI_RIGHT_MOTOR 2
    #define MOTOR_2_CHANNEL D5_PWM
    #define MOTOR_2_PIN     D5_PIN

    #define NUM_SERVOS 1
    #define TRI_REAR_SERVO 0
    #define SERVO_0_CHANNEL D2_PWM
    #define SERVO_0_PIN     D2_PIN

#else
  #error "Need to define an aircraft type"
#endif

void compute_mix(fixedpointnum throttle, fixedpointnum pid[]);
