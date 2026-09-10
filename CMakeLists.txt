# ------------------------------------------------------------------------------
#        MOLA: continuous-time LiDAR-inertial odometry (mola_ct_lio)
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(mola_ct_lio LANGUAGES CXX VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------------------------
# The continuous-time estimator core.
#
# It depends on Eigen alone, so that the trajectory mathematics can be built and
# tested without a ROS or MOLA stack present.
# ------------------------------------------------------------------------------
add_library(mola_ct_lio_core
  core/src/CtSegment.cpp
)
target_include_directories(mola_ct_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mola_ct_lio_core PUBLIC Eigen3::Eigen)
set_target_properties(mola_ct_lio_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_ct_jacobians core/test/test_ct_jacobians.cpp)
    target_link_libraries(test_ct_jacobians mola_ct_lio_core)
  endif()

  install(DIRECTORY core/include/ DESTINATION include)
  install(TARGETS mola_ct_lio_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_package()
else()
  message(STATUS "ament_cmake not found: building the core library only")
endif()
