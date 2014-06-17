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

#include "lib_fp.h"

// uncrashability mode
#define UNCRASHABLELOOKAHEADTIME FIXEDPOINTONE // look ahead one second to see if we are going to be at a bad altitude
#define UNCRASHABLERECOVERYANGLE FIXEDPOINTCONSTANT(15) // don't let the pilot pitch or roll more than 20 degrees when altitude is too low.
#define FPUNCRASHABLE_RADIUS FIXEDPOINTCONSTANT(UNCRAHSABLE_RADIUS)
#define FPUNCRAHSABLE_MAX_ALTITUDE_OFFSET FIXEDPOINTCONSTANT(UNCRAHSABLE_MAX_ALTITUDE_OFFSET)

void uncrashable(unsigned char gotNewGpsReading, fixedpointnum *angleError, fixedpointnum *throttleOutput);