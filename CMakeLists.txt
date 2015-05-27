cmake_minimum_required(VERSION 2.8.11)
project (TESTSCREWS)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package (Eigen3 REQUIRED)
include_directories (EIGEN3_INCLUDE_DIR)

add_definitions (-DSCREWS_EXPORTS)
IF (WIN32)
  add_definitions (-DML_LIBRARY_EXPORT_ATTRIBUTE="__declspec\(dllexport\)")
  add_definitions (-DML_LIBRARY_IMPORT_ATTRIBUTE="__declspec\(dllimport\)")
ELSEIF (CMAKE_SYSTEM_NAME STREQUAL Linux)
  add_definitions (-DML_LIBRARY_IMPORT_ATTRIBUTE="__attribute__\(\(__visibility__\(\"default\"\)\)\)")
  add_definitions (-DML_LIBRARY_EXPORT_ATTRIBUTE=)
ELSEIF (APPLE)
  add_definitions (-DML_LIBRARY_EXPORT_ATTRIBUTE=)
  add_definitions (-DML_LIBRARY_EXPORT_ATTRIBUTE=)
ENDIF (WIN32)

set (HEADER_FILES 
  src/homogeneousTransform.hpp 
  src/rotation.hpp 
  src/screwException.hpp 
  src/screws.hpp 
  src/screwsInitLibrary.hpp 
  src/translation.hpp 
  src/twist.hpp 
  src/vector6.hpp)

add_library (Screws src/screwException.cpp ${HEADER_FILES})
add_executable (testScrews src/testScrews.cpp)
target_link_libraries (testScrews LINK_PUBLIC Screws)
