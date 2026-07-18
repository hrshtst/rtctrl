include(Catch)

add_executable(unit_tests
  unit/smoke_test.cpp
  unit/model_test.cpp
  unit/trajectory_test.cpp
  unit/ik_test.cpp)
target_link_libraries(unit_tests PRIVATE rtctrl::rtctrl Catch2::Catch2WithMain)
# Test code refers to models/ and config/ by repo-root-relative paths.
catch_discover_tests(unit_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS unit)

add_executable(integration_tests
  integration/sim_arm_test.cpp)
target_link_libraries(integration_tests PRIVATE rtctrl::rtctrl
  Catch2::Catch2WithMain)
catch_discover_tests(integration_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS integration)
