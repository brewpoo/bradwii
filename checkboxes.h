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

#define NUM_POSSIBLE_CHECKBOXES 20 // allocate room for more checkboxes in eeprom memory in case we add them later

#define CHECKBOX_ARM 0
#define CHECKBOX_AUTOTHROTTLE 1
#define CHECKBOX_ALTHOLD 2
#define CHECKBOX_COMPASS 3
#define CHECKBOX_POSITION_HOLD 4
#define CHECKBOX_RETURN_TO_HOME 5
#define CHECKBOX_SEMIACRO 6
#define CHECKBOX_FULLACRO 7
#define CHECKBOX_HIGHRATES 8
#define CHECKBOX_HIGHANGLE 9
#define CHECKBOX_AUTOTUNE 10
#define CHECKBOX_UNCRASHABLE 11
#define CHECKBOX_HEADFREE 12
#define CHECKBOX_AUTOPILOT 13
#define NUM_CHECKBOXES 14

#define CHECKBOX_MASK_ARM (1<<CHECKBOX_ARM)
#define CHECKBOX_MASK_AUTOTHROTTLE (1<<CHECKBOX_AUTOTHROTTLE)
#define CHECKBOX_MASK_ALTHOLD (1<<CHECKBOX_ALTHOLD)
#define CHECKBOX_MASK_COMPASS (1<<CHECKBOX_COMPASS)
#define CHECKBOX_MASK_POSITIONHOLD (1<<CHECKBOX_POSITION_HOLD)
#define CHECKBOX_MASK_RETURNTOHOME (1<<CHECKBOX_RETURN_TO_HOME)
#define CHECKBOX_MASK_SEMIACRO (1<<CHECKBOX_SEMIACRO)
#define CHECKBOX_MASK_FULLACRO (1<<CHECKBOX_FULLACRO)
#define CHECKBOX_MASK_HIGHRATES (1<<CHECKBOX_HIGHRATES)
#define CHECKBOX_MASK_HIGHANGLE (1<<CHECKBOX_HIGHANGLE)
#define CHECKBOX_MASK_AUTOTUNE (1<<CHECKBOX_AUTOTUNE)
#define CHECKBOX_MASK_UNCRASHABLE (1<<CHECKBOX_UNCRASHABLE)
#define CHECKBOX_MASK_HEADFREE (1<<CHECKBOX_HEADFREE)
#define CHECKBOX_MASK_AUTOPILOT (1<<CHECKBOX_AUTOPILOT)

#define CHECKBOX_MASK_AUX1LOW (1<<0)
#define CHECKBOX_MASK_AUX1MID (1<<1)
#define CHECKBOX_MASK_AUX1HIGH (1<<2)
#define CHECKBOX_MASK_AUX2LOW (1<<3)
#define CHECKBOX_MASK_AUX2MID (1<<4)
#define CHECKBOX_MASK_AUX2HIGH (1<<5)
#define CHECKBOX_MASK_AUX3LOW (1<<6)
#define CHECKBOX_MASK_AUX3MID (1<<7)
#define CHECKBOX_MASK_AUX3HIGH (1<<8)
#define CHECKBOX_MASK_AUX4LOW (1<<9)
#define CHECKBOX_MASK_AUX4MID (1<<10)
#define CHECKBOX_MASK_AUX4HIGH (1<<11)

void check_checkbox_items();