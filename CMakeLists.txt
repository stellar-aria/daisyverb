cmake_minimum_required(VERSION 3.26)
cmake_policy(SET CMP0048 NEW)

# Fetch libDaisy
include(FetchContent)
FetchContent_Declare(daisy
  GIT_REPOSITORY https://github.com/stellar-aria/libDaisy
  GIT_TAG 23c5856
)
FetchContent_MakeAvailable(daisy)

# Our project declaration
project(reverb VERSION 0.0.1)

set(FIRMWARE_NAME ${CMAKE_PROJECT_NAME})
set(FIRMWARE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
)

# DaisyProject.cmake uses FIRMWARE_NAME and FIRMWARE_SOURCES to build a target called ${FIRMWARE_NAME}
include(DaisyProject)

target_include_directories(${FIRMWARE_NAME} PUBLIC include)

set_target_properties(${FIRMWARE_NAME} PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED YES
)
