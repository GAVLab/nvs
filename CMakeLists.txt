cmake_minimum_required(VERSION 2.4.6)
project(nvs)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


# Set the default path for built executables/libraries

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH     ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Built type for cmake")

# serial package

if(NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)
    find_package(serial REQUIRED)
else (NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)
    set(serial_FOUND TRUE)
endif(NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)

if(serial_FOUND)
    include_directories(${serial_INCLUDE_DIRS})
    list(APPEND NVS_LINK_LIBS ${serial_LIBRARIES})
    if(UNIX AND NOT APPLE)
        list(APPEND NVS_LINK_LIBS rt)
    endif(UNIX AND NOT APPLE)
else(serial_FOUND)
    message(FATAL_ERROR "Serial library was not found.")
endif(serial_FOUND)

# Add the include folder to the include path

include_directories(${PROJECT_SOURCE_DIR}/include)

# set headers and sources

set(NVS_SRCS src/nvs.cpp src/main.cpp)
set(NVS_HEADERS include/nvs/nvs.h
                include/nvs/nvs_structures.h)

## boost

find_package(Boost COMPONENTS thread system REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND NVS_LINK_LIBS   ${Boost_SYSTEM_LIBRARY}
                            ${Boost_THREAD_LIBRARY})

# compile the library

add_library(nvs ${NVS_SRCS} ${NVS_HEADERS})
target_link_libraries(nvs ${NVS_LINK_LIBS})

# make run file

add_executable(run_nvs src/main.cpp)
target_link_libraries(run_nvs nvs)