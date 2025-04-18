# FlightControl_Demo/test/CMakeLists.txt
#allow debugging
set(CMAKE_BUILD_TYPE Debug)

# Add GoogleTest
add_subdirectory(external/googletest)

# Define the TESTING preprocessor macro for all test builds
add_compile_definitions(TESTING)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include Google Test and Google Mock headers
include_directories(external/googletest/googletest/include)
include_directories(external/googletest/googlemock/include)

# Find all driver directories and add them to the include path
file(GLOB DRIVER_DIRS ${CMAKE_SOURCE_DIR}/lib/*/src)

# Create a library for the mocks
file(GLOB MOCK_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/*.cpp
)

# Add a library of mocks - if there are no source files yet, use an interface library
if(MOCK_SOURCES)
    add_library(mocks ${MOCK_SOURCES})
else()
    add_library(mocks INTERFACE)
endif()

# Add header include directories for mocks - mock directory MUST be first
target_include_directories(mocks BEFORE INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks  # Mocks FIRST
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${DRIVER_DIRS}  # Add all driver directories
)

# Specify test executable
add_executable(unit_tests 
    # Main test runner
    main.cpp
    
    # Kestrel tests
    #unit/Driver_-_Kestrel/KestrelTest.cpp
    #${CMAKE_SOURCE_DIR}/lib/Driver_-_Kestrel/src/Kestrel.cpp
    
    # Li710 tests
    unit/Driver_-_Li710/Li710Test.cpp
    ${CMAKE_SOURCE_DIR}/lib/Driver_-_Li710/src/Li710.cpp
)

# Link against mocks and GoogleTest
target_link_libraries(unit_tests mocks gtest gtest_main gmock)