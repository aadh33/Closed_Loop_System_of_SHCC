# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly

#//cmake_minimum_required(VERSION 3.16)

#//include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#project(step_motor)
cmake_minimum_required(VERSION 3.16)

# Set the project name
project(Firmware_Feb14)

# Set the IDF_PATH variable to the ESP-IDF installation path
set(IDF_PATH "C:\Users\adith\esp\v5.4\esp-idf")

# Include the ESP-IDF project CMake file
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
