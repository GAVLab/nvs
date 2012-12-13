/*
    NVS NV08C-CSM Data Structures
*/
#ifndef NVSSTRUCTURES_H
#define NVSSTRUCTURES_H

#include "stdint.h"

// Maximum size of an NVS log buffer
#define MAX_NOUT_SIZE (5000)

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
    #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
    #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

// Header prepended to NVS Binary messages
#define HDR_CHKSM_LENGTH 2
PACK(
    struct NVSHeader{
        
    });


#endif