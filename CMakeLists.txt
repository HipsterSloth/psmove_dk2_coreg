cmake_minimum_required(VERSION 3.0)

project(PSMOVEDK2)

#Includes
include_directories(${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/include)
include_directories(${PSMOVEDK2_SOURCE_DIR}/3rdparty/LibOVR/Include)
if(MSVC)
    include_directories(${PSMOVEDK2_SOURCE_DIR}/3rdparty/msvc-support/)
endif()

#Sources
set(SOURCES src/psmdk2.c)
if(MSVC)
    list(APPEND SOURCES
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/msvc-support/getopt.c
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/msvc-support/unistd.c)
endif()
add_executable(psmdk2 ${SOURCES})

#Linked libraries
#TODO: Be cleverer about searching for the libraries
#TODO: Libraries for other platforms.
if (MSVC)
    target_link_libraries(psmdk2
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/lib/win64/psmoveapi.lib
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/lib/win64/psmoveapi_tracker.lib
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/LibOVR/Lib/Windows/x64/Release/VS2013/LibOVR.lib)
endif()