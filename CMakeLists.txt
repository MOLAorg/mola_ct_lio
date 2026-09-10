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

# The module layer needs the MOLA/MRPT stack; the core above does not.
find_package(mola_common QUIET)
find_package(mola_metric_maps QUIET)
find_package(mola_imu_preintegration QUIET)
find_package(mp2p_icp_map QUIET)
find_package(mrpt-maps QUIET)

# ------------------------------------------------------------------------------
# The continuous-time estimator core.
#
# It depends on Eigen alone, so that the trajectory mathematics can be built and
# tested without a ROS or MOLA stack present.
# ------------------------------------------------------------------------------
add_library(mola_ct_lio_core
  core/src/CtSegment.cpp
  core/src/CtNormalEquations.cpp
  core/src/ImuFactor.cpp
  core/src/WindowSystem.cpp
  core/src/WindowOptimizer.cpp
)
target_include_directories(mola_ct_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mola_ct_lio_core PUBLIC Eigen3::Eigen)
set_target_properties(mola_ct_lio_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------------------
# The matching layer: the local map and the correspondence queries.
# ------------------------------------------------------------------------------
set(MOLA_CT_LIO_HAS_MODULE FALSE)
if(mola_metric_maps_FOUND AND mola_imu_preintegration_FOUND AND mrpt-maps_FOUND)
  set(MOLA_CT_LIO_HAS_MODULE TRUE)

  add_library(mola_ct_lio_matching
    module/src/CtMapMatcher.cpp
    module/src/CtOdometryEngine.cpp
  )
  target_include_directories(mola_ct_lio_matching PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/src>
  )
  target_link_libraries(mola_ct_lio_matching PUBLIC
    mola_ct_lio_core
    mola::mola_metric_maps
    mola::mola_imu_preintegration
    mrpt::maps
  )
  set_target_properties(mola_ct_lio_matching PROPERTIES POSITION_INDEPENDENT_CODE ON)
else()
  message(STATUS "mola_metric_maps or mrpt-maps not found: skipping the matching layer")
endif()

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_ct_jacobians core/test/test_ct_jacobians.cpp)
    target_link_libraries(test_ct_jacobians mola_ct_lio_core)

    ament_add_gtest(test_ct_normal_equations core/test/test_ct_normal_equations.cpp)
    target_link_libraries(test_ct_normal_equations mola_ct_lio_core)

    ament_add_gtest(test_imu_factor core/test/test_imu_factor.cpp)
    target_link_libraries(test_imu_factor mola_ct_lio_core)

    ament_add_gtest(test_window_system core/test/test_window_system.cpp)
    target_link_libraries(test_window_system mola_ct_lio_core)

    ament_add_gtest(test_window_optimizer core/test/test_window_optimizer.cpp)
    target_link_libraries(test_window_optimizer mola_ct_lio_core)

    if(MOLA_CT_LIO_HAS_MODULE)
      ament_add_gtest(test_map_matcher module/test/test_map_matcher.cpp)
      target_link_libraries(test_map_matcher mola_ct_lio_matching)

      ament_add_gtest(test_odometry_engine module/test/test_odometry_engine.cpp)
      target_link_libraries(test_odometry_engine mola_ct_lio_matching)
    endif()
  endif()

  install(DIRECTORY core/include/ DESTINATION include)
  if(MOLA_CT_LIO_HAS_MODULE)
    install(TARGETS mola_ct_lio_matching
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  install(TARGETS mola_ct_lio_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_package()
else()
  message(STATUS "ament_cmake not found: building the core library only")
endif()
