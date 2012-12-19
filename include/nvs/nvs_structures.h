/*
    NVS NV08C-CSM Data Structures

    Author: Robert Cofield, for GAVLab
*/
#ifndef NVSSTRUCTURES_H
#define NVSSTRUCTURES_H

#include "stdint.h"

/*
    description
*/
#define MAX_NOUT_SIZE      (5000)   // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)
#define MAXCHAN     50  // Maximum number of signal channels
#define MAX_SAT     33  // maximum number of prns - max prn is 32 plus prn 0 is 33


// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
    #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
    #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif






#endif