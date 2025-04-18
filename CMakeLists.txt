# FlightControl_Demo/CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project(FlightControl_Demo_Tests)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add test directory
add_subdirectory(test)