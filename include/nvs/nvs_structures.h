// /*
//     NVS NV08C-CSM Data Structures

//     Author: Robert Cofield, for GAVLab
// */
#ifndef NVSSTRUCTURES_H
#define NVSSTRUCTURES_H

#include "stdint.h"

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
    #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
    #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif


/*
    Messages to send to reciever
*/
#define queryVersionMsg "$GPGPQ,ALVER*31\r\n"
#define queryCOMSettingMsg "$PORZA,1,115200,*7D"


/*
    Enums from messages
*/
enum NMEA_STD_MSGS {
    DTM,
    GBS,
    GGA,
    GLL,
    GNS,
    GSA,
    GSV,
    RMC,
    VTG,
    ZDA,
    Q
};

enum NMEA_PROP_MSGS {
    ALVER,
    PAMOD,
    PASET,
    PKON1,
    POCWT,
    PONAV,
    PONME,
    POPPS,
    POPWR,
    PORST,
    PORZA,
    PORZB,
    PORZD,
    PORZE,
    PORZX,
    POAST
};

enum FIX_TYPE {
    _NA=1,      // Fix Not available
    _2D=2,      // 2D fix
    _3D=3       // 3D fix
};

enum FIX_MODE {
    N,          // Fix not available
    A,          // Autonomous
    D,          // Differential
    E=6           // Estimated (dead reckoning)
};

#endif