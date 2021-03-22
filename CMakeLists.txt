cmake_minimum_required(VERSION 3.10)

# set the project name
project(SVDAG)

OPTION (USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin")

file(GLOB_RECURSE src_files ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

set(SOURCES 
${src_files}
)

include_directories(inc)

# add the executable
add_executable(SVDAG ${SOURCES})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()