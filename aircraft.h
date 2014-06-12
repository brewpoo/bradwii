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

// set aircraft type dependant defines here
#if (AIRCRAFT_CONFIGURATION == OCTOX8) | (AIRCRAFT_CONFIGURATION==OCTOFLATP) | (AIRCRAFT_CONFIGURATION==OCTOFLATX)
  #define NUMMOTORS 8
  #define NUMSERVOS 0
#elif (AIRCRAFT_CONFIGURATION==Y6) | (AIRCRAFT_CONFIGURATION==HEX6)
  #define NUMMOTORS 6
  #define NUMSERVOS 0
#elif (AIRCRAFT_CONFIGURATION==QUADX) | (AIRCRAFT_CONFIGURATION==QUADP) | (AIRCRAFT_CONFIGURATION==Y4) | (AIRCRAFT_CONFIGURATION==VTAIL4)
  #define NUMMOTORS 4
  #define NUMSERVOS 0
#elif (AIRCRAFT_CONFIGURATION==TRI)
  #define NUMMOTORS 3
  #define NUMSERVOS 1
#elif (AIRCRAFT_CONFIGURATION==BI)
  #define NUMMOTORS 2
  #define NUMSERVOS 2
#else
  #error "Need to define an aircraft type"
#endif

