cmake_minimum_required(VERSION 2.8.3)
project(map_utils_matlab)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(MATLAB QUIET)

if (MATLAB_FOUND)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Yaml-cpp REQUIRED)
  find_package(map_utils REQUIRED)
  find_package(yaml_utils REQUIRED)
  find_package(mex_class_wrapper REQUIRED)
  find_package(mex_conversion_utils REQUIRED)

  include_directories(include
    ${MATLAB_INCLUDE_DIR}
    ${map_utils_INCLUDE_DIR}
    ${yaml_utils_INCLUDE_DIR}
    ${mex_class_wrapper_INCLUDE_DIR}
    ${mex_conversion_utils_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
  )

  # MATLAB Macro definitions
  add_definitions(/DMATLAB_MEX_FILE)
  add_definitions(/DMX_COMPAT_32)

  add_library(grid3d_mex SHARED src/grid3d.cc)

  target_Link_libraries(grid3d_mex
    ${MATLAB_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
  )

  if (UNIX AND APPLE)
    set(MEXEXT ".mexmaci64")
  elseif (UNIX)
    set(MEXEXT ".mexa64")
  endif()

  set_target_properties(grid3d_mex PROPERTIES SUFFIX ${MEXEXT} PREFIX "")

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION "include/${PROJECT_NAME}"
    FILES_MATCHING PATTERN "*.h"
  )
endif()
