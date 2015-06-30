cmake_minimum_required(VERSION 3.0)

project(PSMOVEDK2)

#Includes
include_directories(${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/include)
include_directories(${PSMOVEDK2_SOURCE_DIR}/3rdparty/eigen)
if(MSVC)
    include_directories(${PSMOVEDK2_SOURCE_DIR}/3rdparty/LibOVR/Win_0.6.0.1/Include)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(${PSMOVEDK2_SOURCE_DIR}/3rdparty/LibOVR/Mac_0.5.0.1/Include)
endif()

#Sources
set(SOURCES src/psmdk2.cpp)
add_executable(psmdk2 ${SOURCES})

#Linked libraries
#TODO: Be cleverer about searching for the libraries
if (MSVC)
    target_link_libraries(psmdk2
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/lib/win64/psmoveapi.lib
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/lib/win64/psmoveapi_tracker.lib
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/LibOVR/Win_0.6.0.1/Lib/Windows/x64/Release/VS2013/LibOVR.lib)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(psmdk2
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/lib/mac/libpsmoveapi.dylib
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/psmoveapi/lib/mac/libpsmoveapi_tracker.dylib
        ${PSMOVEDK2_SOURCE_DIR}/3rdparty/LibOVR/Mac_0.5.0.1/Lib/Mac/Release/LibOVR.framework)
endif()