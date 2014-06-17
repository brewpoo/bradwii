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

 
#include <stdlib.h>
#include <avr/io.h>

#include <string.h>
#include "lib_serial.h"
#include "lib_timers.h"
#include "lib_fp.h"

#include "bradwii.h"
#include "serial.h"
#include "defs.h"
#include "checkboxes.h"
#include "compass.h"
#include "eeprom.h"
#include "imu.h"
#include "gps.h"

#define MSP_VERSION 0
#define  VERSION  112 // version 1.12


extern globalstruct global;
extern usersettingsstruct usersettings;

extern const char checkboxnames[];
extern unsigned int lib_i2c_error_count;

void serial_init() {
#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT0)
   lib_serial_initport(0,SERIAL_0_BAUD);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT1)
   lib_serial_initport(1,SERIAL_1_BAUD);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT2)
   lib_serial_initport(2,SERIAL_2_BAUD);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT3)
   lib_serial_initport(3,SERIAL_3_BAUD);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORTUSB)
   lib_serial_initport(USBPORTNUMBER,0);
#endif
}

#if (MULTIWII_CONFIG_SERIAL_PORTS!=NOSERIALPORT)
#define SERIALSTATEIDLE 0
#define SERIALSTATEGOTDOLLARSIGN 1
#define SERIALSTATEGOTM 2
#define SERIALSTATEGOTLESSTHANSIGN 3
#define SERIALSTATEGOTDATASIZE 4
#define SERIALSTATEGOTCOMMAND 5
#define SERIALSTAGEGOTPAYLOAD 6

#define CAPABILITES 1 | ((BAROMETER_TYPE!=NO_BAROMETER)<<1) | ((COMPASS_TYPE!=NO_COMPASS)<<2) | ((GPS_TYPE!=NO_GPS)<<3)

// datagram format is $M<[data size][command][data...][checksum]
// response format is $M>[data size][command][data...][checksum]
//                 or $M![data size][command][data...][checksum] on error
unsigned char serialreceivestate[5]={0};
unsigned char serialcommand[5];
unsigned char serialdatasize[5];
unsigned char serialchecksum[5];

void send_and_checksum_character(unsigned char portnumber,unsigned char c) {
    lib_serial_sendchar(portnumber,c);
    serialchecksum[portnumber]^=c;
}
   
void send_and_checksum_data(char portnumber,unsigned char *data,char length) {
   for (int x=0;x<length;++x)
      send_and_checksum_character(portnumber,data[x]);
}
   
void send_and_checksum_int(char portnumber,unsigned int value) {
   send_and_checksum_data(portnumber,(unsigned char *)&value,2);
}
   
void send_and_checksum_long(char portnumber,unsigned long value) {
   send_and_checksum_data(portnumber,(unsigned char *)&value,4);
}
   
void send_good_header(unsigned char portnumber,unsigned char size) {
   lib_serial_sendchar(portnumber,'$');
   lib_serial_sendchar(portnumber,'M');
   lib_serial_sendchar(portnumber,'>');
   lib_serial_sendchar(portnumber,size);
   serialchecksum[portnumber]=size;
   send_and_checksum_character(portnumber,serialcommand[portnumber]);
}
   
void send_error_header(unsigned char portnumber) {
   lib_serial_sendchar(portnumber,'$');
   lib_serial_sendchar(portnumber,'M');
   lib_serial_sendchar(portnumber,'!');
   lib_serial_sendchar(portnumber,0);
   serialchecksum[portnumber]=0;
   send_and_checksum_character(portnumber,serialcommand[portnumber]);
}
   
void evaluate_command(unsigned char portnumber,unsigned char *data) {
    unsigned char command=serialcommand[portnumber];
    
    
    if (command==MSP_IDENT) { // send rx data
        send_good_header(portnumber,7);
        send_and_checksum_character(portnumber,VERSION);
        send_and_checksum_character(portnumber,AIRCRAFT_CONFIGURATION);
        send_and_checksum_character(portnumber,MSP_VERSION);
        for (int x=0;x<4;++x) send_and_checksum_character(portnumber,0); // 32 bit "capability"
    } else if (command==MSP_RC) { // send rx data
        send_good_header(portnumber,16);
        for (int x=0;x<8;++x) {
            int value=0;
            if (x<RX_NUM_CHANNELS) value=((global.rxValues[x]*500L)>>FIXEDPOINTSHIFT)+1500;

            send_and_checksum_data(portnumber,(unsigned char *)&value,2);
        }
    } else if (command==MSP_ATTITUDE) { // send attitude data
        send_good_header(portnumber,6);
        // convert our estimated gravity vector into roll and pitch angles
        int value;
        value=(global.currentEstimatedEulerAttitude[0]*10)>>FIXEDPOINTSHIFT;
        send_and_checksum_data(portnumber,(unsigned char *)&value,2);
        value=(global.currentEstimatedEulerAttitude[1]*10)>>FIXEDPOINTSHIFT;
        send_and_checksum_data(portnumber,(unsigned char *)&value,2);
        value=(global.currentEstimatedEulerAttitude[2])>>FIXEDPOINTSHIFT;
        send_and_checksum_data(portnumber,(unsigned char *)&value,2);
    } else if (command==MSP_ALTITUDE) { // send attitude data
        send_good_header(portnumber,4);
        fixedpointnum fp=(global.altitude*25)>>(FIXEDPOINTSHIFT-2);
        send_and_checksum_data(portnumber,(unsigned char *)&fp,4);
    } else if (command==MSP_MAG_CALIBRATION) { // send attitude data
        if (!global.state.armed) calibrate_compass();
        send_good_header(portnumber,0);
    } else if (command==MSP_ACC_CALIBRATION) { // send attitude data
        if (!global.state.armed) {
            global.state.calibratingAccAndGyro=1;
            calibrate_gyro_and_accelerometer();
        }
        send_good_header(portnumber,0);
    } else if (command==MSP_RAW_IMU) { // send attitude data
        send_good_header(portnumber,18);
        for (int x=0;x<3;++x) { // convert from g's to what multiwii uses
            int value=global.correctedVectorGs[x]>>8;
            send_and_checksum_data(portnumber,(unsigned char *)&value,2);
        }
        for (int x=0;x<3;++x) { // convert from degrees per second to /8000
            int value=(global.gyrorate[x])>>14; // this is aproximate
            send_and_checksum_data(portnumber,(unsigned char *)&value,2);
        }
        for (int x=0;x<3;++x) { // convert from
            int value=(global.compassVector[x])>>8;
            send_and_checksum_data(portnumber,(unsigned char *)&value,2);
        }
    } else if (command==MSP_STATUS) { // send attitude data
        send_good_header(portnumber,10);
        send_and_checksum_int(portnumber,(global.timesliver*15)>>8); // convert from fixedpointnum to microseconds
        send_and_checksum_int(portnumber,lib_i2c_error_count); // i2c error count
        send_and_checksum_int(portnumber,CAPABILITES); // baro mag, gps, sonar
        send_and_checksum_data(portnumber,(unsigned char *)&global.activeCheckboxItems,4); // options1
    } else if (command==MSP_MOTOR) {
        // send motor output data
        send_good_header(portnumber,16);
        for (int x=0;x<8;++x) {
            if (x < NUM_MOTORS) {
                send_and_checksum_int(portnumber,global.motorOutputValue[x]); // current motor value
            } else {
                send_and_checksum_int(portnumber,0);
            }
        }
    } else if (command==MSP_SERVO) {
        // send servo output data
        send_good_header(portnumber,16);
        for (int x=0;x<8;++x) {
            if (x < NUM_SERVOS) {
                send_and_checksum_int(portnumber,global.servoOutputValue[x]); // current servo value
            } else {
                send_and_checksum_int(portnumber,0);
            }
        }
    } else if (command==MSP_PID) {
        // send pid data
      send_good_header(portnumber,3*NUM_PID_ITEMS);
      for (int x=0;x<NUM_PID_ITEMS;++x)
         {
         if (x==ALTITUDE_INDEX)
            send_and_checksum_character(portnumber,usersettings.pid_pgain[x]>>7);
         else if (x==NAVIGATION_INDEX)
            send_and_checksum_character(portnumber,usersettings.pid_pgain[x]>>11);
         else
            send_and_checksum_character(portnumber,usersettings.pid_pgain[x]>>3);
         send_and_checksum_character(portnumber,usersettings.pid_igain[x]);
         if (x==NAVIGATION_INDEX)
            send_and_checksum_character(portnumber,usersettings.pid_dgain[x]>>8);
         else if (x==ALTITUDE_INDEX)
            send_and_checksum_character(portnumber,usersettings.pid_dgain[x]>>9);
         else
            send_and_checksum_character(portnumber,usersettings.pid_dgain[x]>>2);
         }
      }
   else if (command==MSP_SET_PID)
      {
      for (int x=0;x<NUM_PID_ITEMS;++x)
         {
         if (x==ALTITUDE_INDEX)
            usersettings.pid_pgain[x]=((fixedpointnum)(*data++))<<7;
         else if (x==NAVIGATION_INDEX)
            usersettings.pid_pgain[x]=((fixedpointnum)(*data++))<<11;
         else
            usersettings.pid_pgain[x]=((fixedpointnum)(*data++))<<3;
         usersettings.pid_igain[x]=((fixedpointnum)(*data++));
         if (x==NAVIGATION_INDEX)
            usersettings.pid_dgain[x]=((fixedpointnum)(*data++))<<8;
         else if (x==ALTITUDE_INDEX)
            usersettings.pid_dgain[x]=((fixedpointnum)(*data++))<<9;
         else
            usersettings.pid_dgain[x]=((fixedpointnum)(*data++))<<2;

         }
// while testing, make roll pid equal to pitch pid so I only have to change one thing.
//usersettings.pid_pgain[ROLL_INDEX]=usersettings.pid_pgain[PITCH_INDEX];
//usersettings.pid_igain[ROLL_INDEX]=usersettings.pid_igain[PITCH_INDEX];
//usersettings.pid_dgain[ROLL_INDEX]=usersettings.pid_dgain[PITCH_INDEX];
      send_good_header(portnumber,0);
      }
   else if (command==MSP_DEBUG)
      { // send attitude data
      send_good_header(portnumber,8);
      for (int x=0;x<4;++x)
         { // convert from g's to what multiwii uses
         int value=global.debugValue[x];
         send_and_checksum_data(portnumber,(unsigned char *)&value,2);
         }
      }
   else if (command==MSP_BOXNAMES)
      { // send names of checkboxes
      char length=strlen(checkboxnames);
      send_good_header(portnumber,length);
      send_and_checksum_data(portnumber,(unsigned char *)checkboxnames,length);
      }
   else if (command==MSP_SET_BOX)
      { // receive check box settings
      unsigned char *ptr=(unsigned char *)usersettings.checkboxConfiguration;
      for (int x=0;x<NUM_CHECKBOXES*2;++x)
         {
         *ptr++=*data++;
         }
      }
   else if (command==MSP_BOX)
      { // send check box settings
      send_good_header(portnumber,NUM_CHECKBOXES*2);
      send_and_checksum_data(portnumber,(unsigned char *)usersettings.checkboxConfiguration,NUM_CHECKBOXES*2);
      }
   else if (command==MSP_RESET_CONF)
      { // reset user settings
      send_good_header(portnumber,0);
      default_user_settings();
      }
   else if (command==MSP_EEPROM_WRITE)
      { // reset user settings
      send_good_header(portnumber,0);
      if (!global.state.armed) write_user_settings_to_eeprom();
      }
   else if (command==MSP_RAW_GPS)
      { // reset user settings
      send_good_header(portnumber,14);
      send_and_checksum_character(portnumber,0); // gps fix
      send_and_checksum_character(portnumber, global.gps.numSatelites);
      send_and_checksum_long(portnumber,lib_fp_multiply(global.gps.currentLatitude,156250L)); //156250L is 10,000,000L>>LATLONGEXTRASHIFT);
      send_and_checksum_long(portnumber,lib_fp_multiply(global.gps.currentLongitude,156250L));
      send_and_checksum_int(portnumber,global.gps.currentAltitude>>FIXEDPOINTSHIFT); // gps altitude
      send_and_checksum_int(portnumber,(global.gps.currentSpeed*100)>>FIXEDPOINTSHIFT); // gps speed
      }
   else if (command==MSP_COMP_GPS)
      { // reset user settings
      send_good_header(portnumber,5);
      send_and_checksum_int(portnumber,(global.navigationDistance)>>FIXEDPOINTSHIFT); 
      send_and_checksum_int(portnumber,(global.navigationBearing)>>FIXEDPOINTSHIFT); 
      send_and_checksum_character(portnumber,0); // gps update
      }
   else if (command==MSP_RC_TUNING)
      { // user settings
      send_good_header(portnumber,7);
      send_and_checksum_character(portnumber,0); // rcRate
      send_and_checksum_character(portnumber,0); // rcExpo
      send_and_checksum_character(portnumber,usersettings.maxPitchAndRollRate>>(FIXEDPOINTSHIFT+3)); // rollPitchRate
      send_and_checksum_character(portnumber,usersettings.maxYawRate>>(FIXEDPOINTSHIFT+2)); // yawRate
      send_and_checksum_character(portnumber,0); // dynThrPID
      send_and_checksum_character(portnumber,0); // thrMid8
      send_and_checksum_character(portnumber,0); // thrExpo8
      }
   else if (command==MSP_SET_RC_TUNING)
      { // user settings
      data++; //rcRate
      data++; //rcExpo
      usersettings.maxPitchAndRollRate=((fixedpointnum)(*data++))<<(FIXEDPOINTSHIFT+3); // rollPitchRate
      usersettings.maxYawRate=((fixedpointnum)(*data++))<<(FIXEDPOINTSHIFT+2); // yawRate
      data++; // dynThrPID
      data++; // thrMid8
      data++; // thrExpo8
      send_good_header(portnumber,0);
      }

   else // we don't know this command
      {
      send_error_header(portnumber);
      }
   lib_serial_sendchar(portnumber,serialchecksum[portnumber]);
   }

#define MAXPAYLOADSIZE 64

void serial_check_port_for_action(unsigned char portnumber) {
    int numcharsavailable;
    while ((numcharsavailable=lib_serial_numcharsavailable(portnumber))) {
        if (serialreceivestate[portnumber]==SERIALSTATEGOTCOMMAND) {
            // this is the only state where we have to read more than one byte, so do this first, even though it's not first in the sequence of events
            // we need to wait for data plus the checksum.  But don't process until we have enough space in the output buffer
            int spaceneeded=40;
            if (serialcommand[portnumber]==MSP_BOXNAMES) spaceneeded=strlen(checkboxnames)+10;

            if (numcharsavailable>serialdatasize[portnumber] && lib_serial_availableoutputbuffersize(portnumber)>=spaceneeded) {
                unsigned char data[MAXPAYLOADSIZE+1];
                lib_serial_getdata(portnumber, data, serialdatasize[portnumber]+1);
            for (int x=0;x<serialdatasize[portnumber];++x)
               serialchecksum[portnumber]^=data[x];
            if (serialchecksum[portnumber]==data[serialdatasize[portnumber]]) {
               evaluate_command(portnumber,data);
            }
                serialreceivestate[portnumber]=SERIALSTATEIDLE;
            } else return;
        } else {
            unsigned char c=lib_serial_getchar(portnumber);
         
            if (serialreceivestate[portnumber]==SERIALSTATEIDLE) {
                if (c=='$') serialreceivestate[portnumber]=SERIALSTATEGOTDOLLARSIGN;
            } else if (serialreceivestate[portnumber]==SERIALSTATEGOTDOLLARSIGN) {
                if (c=='M') serialreceivestate[portnumber]=SERIALSTATEGOTM;
                else serialreceivestate[portnumber]=SERIALSTATEIDLE;
            } else if (serialreceivestate[portnumber]==SERIALSTATEGOTM) {
                if (c=='<') serialreceivestate[portnumber]=SERIALSTATEGOTLESSTHANSIGN;
                else serialreceivestate[portnumber]=SERIALSTATEIDLE;
            } else if (serialreceivestate[portnumber]==SERIALSTATEGOTLESSTHANSIGN) {
                serialdatasize[portnumber]=c;
                if (c>MAXPAYLOADSIZE) serialreceivestate[portnumber]=SERIALSTATEIDLE;
                else {
                    serialchecksum[portnumber]=c;
                    serialreceivestate[portnumber]=SERIALSTATEGOTDATASIZE;
                }
            } else if (serialreceivestate[portnumber]==SERIALSTATEGOTDATASIZE) {
                serialcommand[portnumber]=c;
                serialchecksum[portnumber]^=c;
                serialreceivestate[portnumber]=SERIALSTATEGOTCOMMAND;
            }
        }
    }
}

//#define SERIALTEXTDEBUG
#ifdef SERIALTEXTDEBUG
void serial_print_number(char portnumber,long num,int digits,int decimals,char usebuffer) {
    // prints a int number, right justified, using digits # of digits, puting a
    // decimal decimals places from the end, and using blank
    // to fill all blank spaces

    char stg[12];
    char *ptr;
    int x;

    ptr=stg+11;
   
    *ptr='\0';
    if (num<0) {
        num=-num;
        *(--ptr)='-';
    } else {
      *(--ptr)=' ';
    }
      
    for (x=1;x<=digits;++x) {
        if (num==0) *(--ptr)=' ';
        else {
            *(--ptr)=48+num-(num/10)*10;
            num/=10;
        }
        if (x==decimals) *(--ptr)='.';
    }
    lib_serial_sendstring(portnumber,ptr);
}

void serial_print_fixedpoint(char portnumber,fixedpointnum fp) {
    serial_print_number(portnumber,lib_fp_multiply(fp,1000),7,3,1);
    lib_serial_sendstring(portnumber,"\n\r");
}
   
void serial_check_port_for_actiontest(char portnumber) {
    int numcharsavailable=lib_serial_numcharsavailable(portnumber);
    if (numcharsavailable) {
        char c=lib_serial_getchar(portnumber);
        lib_serial_sendstring(portnumber,"got char\n\r");
      
        if (c=='r') {
            // receiver values
            for (int x=0;x<6;++x) {
                serial_print_fixedpoint(portnumber,global.rxValues[x]);
            }
        } else if (c=='g') {
             // gyro values
            for (int x=0;x<3;++x) {
                serial_print_fixedpoint(portnumber,global.gyrorate[x]);
            }
        } else if (c=='a') {
            // acc g values
            for (int x=0;x<3;++x) {
                serial_print_fixedpoint(portnumber,global.correctedVectorGs[x]);
            }
        } else if (c=='t') {
            // atttude angle values
            for (int x=0;x<3;++x) {
                serial_print_fixedpoint(portnumber,global.currentEstimatedEulerAttitude[x]);
            }
         } else if (c=='e') {
             // atttude angle values
             serial_print_fixedpoint(portnumber,global.estimatedDownVector[0]);
             serial_print_fixedpoint(portnumber,global.estimatedDownVector[1]);
             serial_print_fixedpoint(portnumber,global.estimatedDownVector[2]);
             serial_print_fixedpoint(portnumber,global.estimatedWestVector[0]);
             serial_print_fixedpoint(portnumber,global.estimatedWestVector[1]);
             serial_print_fixedpoint(portnumber,global.estimatedWestVector[2]);
         } else if (c=='d') {
             // debug values
             for (int x=0;x<3;++x)
                 serial_print_fixedpoint(portnumber,global.debugValue[x]);
         } else if (c=='l') {
             // altitude
             serial_print_fixedpoint(portnumber,global.altitude);
         } else if (c=='p') {
             // atttude angle values
             serial_print_fixedpoint(portnumber,usersettings.pid_pgain[0]);
             serial_print_fixedpoint(portnumber,usersettings.pid_igain[0]);
             serial_print_fixedpoint(portnumber,usersettings.pid_dgain[0]);
         }
        lib_serial_sendstring(portnumber,"\n\r");
    }
}
#endif
#endif

void serial_check_for_action() {
    // to be called by the main program every cycle so that we can check to see if we need to respond to incoming characters
#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT0)
   serial_check_port_for_action(0);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT1)
   serial_check_port_for_action(1);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT2)
   serial_check_port_for_action(2);
//   serial_check_port_for_actiontest(2);
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORT3)
#ifdef SERIALTEXTDEBUG
   serial_check_port_for_actiontest(3);
#else
   serial_check_port_for_action(3);
#endif
#endif

#if (MULTIWII_CONFIG_SERIAL_PORTS & SERIALPORTUSB)
   serial_check_port_for_action(USBPORTNUMBER);
#endif
}
