/*
    NVS NV08C-CSM Data Structures

    Author: Robert Cofield, for GAVLab
*/
#ifndef NVSSTRUCTURES_H
#define NVSSTRUCTURES_H

#include "stdint.h"

/*
    limits to impose
*/
#define MAX_NOUT_SIZE      (5000)   // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)
#define MAXCHAN     50  // Maximum number of signal channels
#define MAX_SAT     33  // maximum number of prns - max prn is 32 plus prn 0 is 33
#define MAX_MSG_SIZE (50) // max for the single message buffer    



// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
    #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
    #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

/*
 *  Messages to send to receiver
 */
const static std::string setBINRMsgNMEA = "$PORZA,0,115200,4*79\r\n"; // Request setting binary protocols
// chapter 4.3 -  sets binr, baud rates, port number 1
const static uint8_t setBINRMsgBINR[10] = {0x10, 0x0b, 0x31, 0x00,0xc2,0x01,0x00, 0x04, 0x10,0x03};
// chapter 36.2 - set binr, nothing else
const static uint8_t setBINRMsgBINR_[4] = {0x10, 0xb2, 0x10,0x03};
// chapter 36.3 - set binr, and some required parameters
const static uint8_t setBINRMsgBINR__[6] = {0x10, 0xb2, 0x0e,0x00, 0x10,0x03};

const static uint8_t reqParamMsg[4] = {0x10, 0x0d, 0x10, 0x03};
const static uint8_t reqSilenceMsg[4] = {0x10, 0x0e, 0x10, 0x03};
const static uint8_t reqVersionMsg[4] = {0x10, 0x1b, 0x10, 0x03};
const static uint8_t reqTestMsg[5] = {0x10, 0x11, 0x00, 0x10, 0x03};



#endif