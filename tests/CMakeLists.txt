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
  unit/command_current_test.cpp
  unit/vendor_gravity_test.cpp
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

# SI profile flags: strict nonnegative doubles; raw and SI forms of
# one register are mutually exclusive. Both reject before any bus
# contact.
add_test(NAME x7_set_param_rejects_negative_si
  COMMAND x7_set_param --profile-vel-si -1.0)
set_tests_properties(x7_set_param_rejects_negative_si PROPERTIES
  PASS_REGULAR_EXPRESSION "invalid value"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_set_param_rejects_raw_and_si
  COMMAND x7_set_param --profile-vel 100 --profile-vel-si 1.0)
set_tests_properties(x7_set_param_rejects_raw_and_si PROPERTIES
  PASS_REGULAR_EXPRESSION "not both"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)

# x7_float --log must fail fast, BEFORE any bus contact: a missing
# value, a flag-looking value ("--log --bogus" must not create
# ./--bogus and proceed — review finding), and an unopenable path all
# reject without a session (no "bus:" line).
add_test(NAME x7_float_rejects_valueless_log COMMAND x7_float --log)
set_tests_properties(x7_float_rejects_valueless_log PROPERTIES
  PASS_REGULAR_EXPRESSION "--log requires a value" LABELS unit)
add_test(NAME x7_float_rejects_flag_as_log_value
  COMMAND x7_float --log --bogus)
set_tests_properties(x7_float_rejects_flag_as_log_value PROPERTIES
  PASS_REGULAR_EXPRESSION "--log requires a value" LABELS unit)
add_test(NAME x7_track_rejects_flag_as_log_value
  COMMAND x7_track --log --bogus)
set_tests_properties(x7_track_rejects_flag_as_log_value PROPERTIES
  PASS_REGULAR_EXPRESSION "--log requires a value" LABELS unit)
add_test(NAME x7_float_rejects_unopenable_log
  COMMAND x7_float --log /nonexistent-rtctrl-dir/f.csv 20)
set_tests_properties(x7_float_rejects_unopenable_log PROPERTIES
  PASS_REGULAR_EXPRESSION "cannot open log file"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# The marker-anchored protocol needs deadline + window; a shorter run
# would bypass the marker guarantee (review finding).
add_test(NAME x7_float_rejects_short_duration COMMAND x7_float 2)
set_tests_properties(x7_float_rejects_short_duration PROPERTIES
  PASS_REGULAR_EXPRESSION "shorter than the marker deadline"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# 13-14 s would fit deadline+window with ZERO margin — the outer
# deadline could truncate an edge-started window (review finding); the
# enforced minimum is 15 s.
add_test(NAME x7_float_rejects_marginless_duration COMMAND x7_float 14)
set_tests_properties(x7_float_rejects_marginless_duration PROPERTIES
  PASS_REGULAR_EXPRESSION "shorter than the marker deadline"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# A malformed command_torque_scale must reject before any bus contact
# at the APPLICATION level too (review finding).
add_test(NAME x7_float_rejects_malformed_scale
  COMMAND x7_float --config tests/data/crane_x7_bad_scale.toml 20
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_rejects_malformed_scale PROPERTIES
  PASS_REGULAR_EXPRESSION "command_torque_scale"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)

# End-to-end log validation over the pty emulator: structure, event
# semantics, and produced rows (check_float_log.py).
add_test(NAME x7_float_log_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/float_log_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_float>
          ${CMAKE_BINARY_DIR}/float_log_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_log_smoke PROPERTIES LABELS integration)

# EFL study smoke: one held-out case end-to-end. --case output must
# declare itself NONCANONICAL — only the complete unfiltered matrix
# from a clean tree may be promoted to data/efl_study/results.json.
add_test(NAME x7_efl_study_smoke
  COMMAND x7_efl_study --case F4 --out ${CMAKE_BINARY_DIR}/efl_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_efl_study_smoke PROPERTIES
  PASS_REGULAR_EXPRESSION "NONCANONICAL" LABELS integration)
# ...and the JSON itself must be well-formed, explicitly noncanonical,
# and schema-complete (review finding: the stdout regex alone would
# pass on malformed output).
add_test(NAME x7_efl_study_smoke_json
  COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/integration/efl_smoke_check.py
          ${CMAKE_BINARY_DIR}/efl_smoke/results.json)
set_tests_properties(x7_efl_study_smoke_json PROPERTIES
  DEPENDS x7_efl_study_smoke LABELS integration)
# --zvs writes one rk_anim motion file per SimArm-based run (passive
# logging). 14 = the 13 development-grid runs plus R1's PRACTICAL cell
# (the acceleration-domain R1 cells are not-run under the recorded
# no-surviving-candidate outcome; the TwoMassArm screens never emit).
add_test(NAME x7_efl_study_zvs_smoke
  COMMAND x7_efl_study --case R1 --zvs
          --out ${CMAKE_BINARY_DIR}/efl_zvs_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_efl_study_zvs_smoke PROPERTIES
  PASS_REGULAR_EXPRESSION "zvs motion files: 14" LABELS integration)

# Feel-mode operator-safety gates (review findings): duration bounded
# to 60 s, and the known-failed all-1.0 configuration refused — both
# BEFORE any bus contact.
add_test(NAME x7_float_rejects_long_feel COMMAND x7_float --feel 90)
set_tests_properties(x7_float_rejects_long_feel PROPERTIES
  PASS_REGULAR_EXPRESSION "exceeds the reviewed"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# ...and the bound is GLOBAL: a huge acceptance duration must not
# reach the runner's floating-to-long cycle conversion (review
# finding).
add_test(NAME x7_float_rejects_long_acceptance COMMAND x7_float 100000)
set_tests_properties(x7_float_rejects_long_acceptance PROPERTIES
  PASS_REGULAR_EXPRESSION "exceeds the reviewed"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_float_rejects_uncalibrated_feel
  COMMAND x7_float --feel 30
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_rejects_uncalibrated_feel PROPERTIES
  PASS_REGULAR_EXPRESSION "APPROVED vendor calibration"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# near-1 scales pass Config validation but are effectively the failed
# calibration — feel mode requires the EXACT approved vector (review
# finding: a '< 1.0' gate accepted 0.999).
add_test(NAME x7_float_rejects_near_one_feel
  COMMAND x7_float --config tests/data/crane_x7_near_one_scale.toml
          --feel 30
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_rejects_near_one_feel PROPERTIES
  PASS_REGULAR_EXPRESSION "APPROVED vendor calibration"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# Since the 2026-07-31 un-parking decision the vendor-calibration
# gate is MODE-INDEPENDENT: default-scale adoption was declined, so
# the repo default config (all-1.0, the known-failed float
# configuration) must be refused in acceptance mode too — before any
# bus contact.
add_test(NAME x7_float_rejects_uncalibrated_acceptance
  COMMAND x7_float 30
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_rejects_uncalibrated_acceptance PROPERTIES
  PASS_REGULAR_EXPRESSION "APPROVED vendor calibration"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
