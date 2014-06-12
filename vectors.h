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

#pragma once

#include "lib_fp.h"

// an attitude is defined as first a rotation from north, then a 3d vector that points down (gravity) from the rotated reference.
typedef struct {
    fixedpointnum westVector[3];
    fixedpointnum downVector[3];
} attitudestruct;

void vector_cross_product(fixedpointnum *v1,fixedpointnum *v2,fixedpointnum *v3);
fixedpointnum normalize_vector(fixedpointnum *v);
fixedpointnum vector_dot_product(fixedpointnum *v1,fixedpointnum *v2);
void vector_difference_to_euler_angles(fixedpointnum *v1, fixedpointnum *v2, fixedpointnum *euler);
void attitude_to_euler_angles(attitudestruct *theattitude,fixedpointnum *eulerangles);
void rotate_vector_with_small_angles(fixedpointnum *v,fixedpointnum rolldeltaangle, fixedpointnum pitchdeltaangle, fixedpointnum yawdeltaangle);
void rotate_vector_by_axis_angle(fixedpointnum *v1, fixedpointnum *axisvector, fixedpointnum angle, fixedpointnum *v2);
void rotate_vector_by_axis_small_angle(fixedpointnum *v1, fixedpointnum *axisvector, fixedpointnum angle);
