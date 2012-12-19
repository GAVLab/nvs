find_path(nvs_INCLUDE_DIRS ublox/ublox.h /usr/include 
          /usr/local/include "$ENV{NAMER_ROOT}")

find_library(nvs_LIBRARIES ublox /usr/lib /usr/local/lib
             "$ENV{NAMER_ROOT}")

set(NVS_FOUND TRUE)

if (NOT ublox_INCLUDE_DIRS)
    set(ublox_FOUND FALSE)
endif (NOT ublox_INCLUDE_DIRS)

if (NOT ublox_LIBRARIES)
    set(ublox_FOUND FALSE)
endif (NOT ublox_LIBRARIES)