cmake_minimum_required(VERSION 3.0.0)
project(GTSAM_Practice)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -pthread")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3 -fopenmp -pthread")

find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

set(exec_sources
    src/2dOdometry.cpp
     # Add more .cpp files as needed
)

#Build each executable
foreach(source_file ${exec_sources})
    get_filename_component(exec_name ${source_file} NAME_WE) 
    add_executable(${exec_name} ${source_file})
    target_link_libraries(${exec_name} gtsam)

    # Create a custom target to run the executable
    add_custom_target(run_${exec_name}
                      COMMAND ./${exec_name}
                      DEPENDS ${exec_name}
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                      COMMENT "Running ${exec_name}")
endforeach()

# Create a custom target to build all executables
add_custom_target(build_all_executables ALL
                  DEPENDS ${exec_sources}
                  COMMENT "Building all executables")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
