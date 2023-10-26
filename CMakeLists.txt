#
# Copyright 2022 SignalCraft Technologies Inc.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

### Project Setup #############################################################
cmake_minimum_required(VERSION 3.5.1)
project(SC2430 CXX)

### Configure Compiler ########################################################
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD" AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_EXE_LINKER_FLAGS "-lthr ${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}")
endif()

### Set up build environment ##################################################
option(UHD_USE_STATIC_LIBS OFF)
find_package(UHD 4.3.0 REQUIRED)

set(UHD_BOOST_REQUIRED_COMPONENTS
    program_options
    system
    thread
)
set(BOOST_MIN_VERSION 1.71)
include(UHDBoost)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(lib)

list(APPEND sc2430_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/sc2430_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/sc2430_phy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/sc2430_expert.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/filter_specification.cpp
)
add_subdirectory(include)

message(STATUS "UHD Module Path (to which the extension will be linked):")
message(STATUS ${UHD_MODULE_PATH})

### Make the executable #######################################################
add_library(sc2430 SHARED ${sc2430_sources})

### Install ###################################################################
set(extension_target ${CMAKE_INSTALL_PREFIX}/lib/libsc2430.so)

install(TARGETS sc2430 DESTINATION lib)
install(CODE "execute_process(COMMAND bash -c \"mkdir -p ${UHD_MODULE_PATH} && ln -s -f ${extension_target} -t ${UHD_MODULE_PATH}\")")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include FILES_MATCHING PATTERN "*.h*")
