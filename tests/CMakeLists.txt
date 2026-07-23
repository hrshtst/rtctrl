include(Catch)

add_executable(unit_tests
  unit/smoke_test.cpp
  unit/model_test.cpp
  unit/trajectory_test.cpp
  unit/ik_test.cpp
  unit/dxl_test.cpp
  unit/hw_test.cpp
  unit/hw_modes_test.cpp
  unit/gravity_test.cpp
  unit/runner_test.cpp
  unit/computed_torque_test.cpp
  unit/track_common_test.cpp
  unit/two_mass_arm_test.cpp)
target_link_libraries(unit_tests PRIVATE rtctrl::rtctrl Catch2::Catch2WithMain)
# track_common_test exercises the app-layer orchestration (settle gate,
# TrackingRun latency verification) directly from apps/ — headers only.
target_include_directories(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/apps)
# Test code refers to models/ and config/ by repo-root-relative paths.
catch_discover_tests(unit_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS unit)

add_executable(integration_tests
  integration/sim_arm_test.cpp
  integration/pty_test.cpp
  integration/thread_test.cpp
  integration/gravity_sim_test.cpp
  integration/tracking_sim_test.cpp)
target_link_libraries(integration_tests PRIVATE rtctrl::rtctrl
  Catch2::Catch2WithMain)
catch_discover_tests(integration_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS integration)
