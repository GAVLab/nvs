cmake_minimum_required(VERSION 2.4.6)
project(nvs)

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Built type for cmake")


if(NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)
    find_package(serial REQUIRED)
else (NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)
    set(serial_FOUND TRUE)
endif(NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)

# Find the serial package
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
set(NVS_SRCS src/nvs.cpp)
set(NVS_HEADERS include/nvs/nvs.h
                include/nvs/nvs_structures.h)


# Find boost
find_package(Boost COMPONENTS thread system REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

list(APPEND NVS_LINK_LIBS ${Boost_SYSTEM_LIBRARY}
                           ${Boost_THREAD_LIBRARY})


# compile the library
add_library(nvs ${NVS_SRCS} ${NVS_HEADERS})

add_executable(run_nvs src/main.cpp)

#SET(CMAKE_INSTALL_PREFIX /usr/local)

INSTALL(TARGETS nvs
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)

INSTALL(FILES include/nvs/nvs.h include/nvs/nvs_structures.h
        DESTINATION include/nvs)

INSTALL(FILES Findnvs.cmake
        DESTINATION ${CMAKE_FIND_INSTALL_PATH}/Modules/)