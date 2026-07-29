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
  unit/two_mass_arm_test.cpp
  unit/lagged_arm_test.cpp
  unit/efl_test.cpp
  unit/study_metrics_test.cpp
  unit/session_watchdog_test.cpp
  unit/ident_common_test.cpp
  unit/x7_common_test.cpp)
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
  integration/tracking_sim_test.cpp
  integration/ident_sim_test.cpp
  integration/quiesce_pty_test.cpp)
target_link_libraries(integration_tests PRIVATE rtctrl::rtctrl
  Catch2::Catch2WithMain)
# ident_sim_test drives the app-layer IdentRun and TwoMassArm fixture.
target_include_directories(integration_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/apps)
catch_discover_tests(integration_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS integration)

# CLI narrowing regression (review finding): a --joint value beyond
# int range truncated into a VALID joint index (4294967296 -> 0) and
# the app proceeded; it must reject before any configuration loading.
add_test(NAME x7_ident_rejects_oversized_joint
  COMMAND x7_ident --joint 4294967296)
set_tests_properties(x7_ident_rejects_oversized_joint PROPERTIES
  PASS_REGULAR_EXPRESSION "--joint rejected" LABELS unit)

# Dataset-integrity regressions for the offline analysis (frozen vs
# live-integrator merge refusal, frozen-bias record validity). The
# build-local uv cache keeps the test hermetic — restricted
# environments cannot write ~/.cache/uv (review finding).
add_test(NAME ident_analysis_merge_guards
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_ident_analysis.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(ident_analysis_merge_guards PROPERTIES LABELS unit)
