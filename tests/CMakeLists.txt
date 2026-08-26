include(Catch)

add_executable(unit_tests
  unit/smoke_test.cpp
  unit/model_test.cpp
  unit/trajectory_test.cpp
  unit/ptp_planner_test.cpp
  unit/ptp_config_test.cpp
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
  unit/ident_common_test.cpp
  unit/gravity_demo_test.cpp
  unit/bringup_test.cpp
  unit/dxl_parameters_test.cpp
  unit/x7_common_test.cpp)
target_link_libraries(unit_tests PRIVATE rtctrl::rtctrl Catch2::Catch2WithMain)
# track_common_test exercises the app-layer orchestration (settle gate,
# TrackingRun latency verification) directly from apps/ — headers only.
target_include_directories(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/apps)
# Test code refers to models/ and config/ by repo-root-relative paths.
catch_discover_tests(unit_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS unit)

add_test(NAME x7_plan_ptp_generates_example
  COMMAND x7_plan_ptp --config config/ptp_example.toml
          --output build/test_ptp_example.zvs --motion-time 0.2)
set_tests_properties(x7_plan_ptp_generates_example PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "wrote .* frames"
  LABELS unit)

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
# The log open now happens after the vendor-calibration gate, so
# these three run the vendor config from the repo root to reach it.
# All still fail BEFORE any bus contact.
add_test(NAME x7_float_rejects_unopenable_log
  COMMAND x7_float --config config/crane_x7_vendor_scale.toml
          --log /nonexistent-rtctrl-dir/f.csv 20
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_rejects_unopenable_log PROPERTIES
  PASS_REGULAR_EXPRESSION "cannot open log file"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# --log is REQUIRED (un-parking condition, review finding: the
# stated unique-filename rule was advisory) ...
add_test(NAME x7_float_requires_log
  COMMAND x7_float --config config/crane_x7_vendor_scale.toml 20
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_requires_log PROPERTIES
  PASS_REGULAR_EXPRESSION "--log is REQUIRED"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# ...and created EXCLUSIVELY: an existing file (here a stable repo
# file, which "wx" can never truncate) is refused, not overwritten.
add_test(NAME x7_float_rejects_existing_log
  COMMAND x7_float --config config/crane_x7_vendor_scale.toml
          --log README.md 20
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_rejects_existing_log PROPERTIES
  PASS_REGULAR_EXPRESSION "already exists"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# The marker-anchored protocol needs deadline + window; a shorter run
# would bypass the marker guarantee (review finding).
add_test(NAME x7_float_rejects_short_duration COMMAND x7_float 2)
set_tests_properties(x7_float_rejects_short_duration PROPERTIES
  PASS_REGULAR_EXPRESSION "shorter than the marker deadline"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# 18-19 s would fit deadline+window with at most one second of margin —
# the outer deadline could truncate an edge-started window (review
# finding); the enforced minimum is 20 s.
add_test(NAME x7_float_rejects_marginless_duration COMMAND x7_float 19)
set_tests_properties(x7_float_rejects_marginless_duration PROPERTIES
  PASS_REGULAR_EXPRESSION "shorter than the marker deadline"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# Demonstration evaluation timing is strict, bounded, and normal-mode
# only. Every rejection happens before config loading or bus contact.
add_test(NAME x7_float_rejects_valueless_evaluation_time
  COMMAND x7_float --evaluation-time)
set_tests_properties(x7_float_rejects_valueless_evaluation_time PROPERTIES
  PASS_REGULAR_EXPRESSION "--evaluation-time requires a value"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_float_rejects_malformed_evaluation_time
  COMMAND x7_float --evaluation-time nan)
set_tests_properties(x7_float_rejects_malformed_evaluation_time PROPERTIES
  PASS_REGULAR_EXPRESSION "invalid evaluation time"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_float_rejects_out_of_range_evaluation_time
  COMMAND x7_float --evaluation-time 51 60)
set_tests_properties(x7_float_rejects_out_of_range_evaluation_time PROPERTIES
  PASS_REGULAR_EXPRESSION "outside the reviewed"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_float_reports_exact_low_evaluation_time
  COMMAND x7_float --evaluation-time 4.99 20)
set_tests_properties(x7_float_reports_exact_low_evaluation_time PROPERTIES
  PASS_REGULAR_EXPRESSION "evaluation time 4.99 s is outside"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_float_reports_exact_high_evaluation_time
  COMMAND x7_float --evaluation-time 50.01 60)
set_tests_properties(x7_float_reports_exact_high_evaluation_time PROPERTIES
  PASS_REGULAR_EXPRESSION "evaluation time 50.01 s is outside"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_float_rejects_evaluation_time_with_feel
  COMMAND x7_float --feel --evaluation-time 10 20)
set_tests_properties(x7_float_rejects_evaluation_time_with_feel PROPERTIES
  PASS_REGULAR_EXPRESSION "not used with --feel"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_float_rejects_short_custom_evaluation_deadline
  COMMAND x7_float --evaluation-time 20 29)
set_tests_properties(x7_float_rejects_short_custom_evaluation_deadline PROPERTIES
  PASS_REGULAR_EXPRESSION "30.0 s minimum"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# A valid longer demonstration window clears timing validation and
# reaches the next pre-bus gate (the required log), without hardware.
add_test(NAME x7_float_accepts_longer_evaluation_time_contract
  COMMAND x7_float --config config/crane_x7_vendor_scale.toml
          --evaluation-time 20 30)
set_tests_properties(x7_float_accepts_longer_evaluation_time_contract PROPERTIES
  PASS_REGULAR_EXPRESSION "--log is REQUIRED"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
# A malformed command_torque_scale must reject before any bus contact
# at the APPLICATION level too (review finding).
add_test(NAME x7_float_rejects_malformed_scale
  COMMAND x7_float --config tests/data/crane_x7_bad_scale.toml 20
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_rejects_malformed_scale PROPERTIES
  PASS_REGULAR_EXPRESSION "command_torque_scale"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)

# x7_gravity_demo (the simplified demonstration app): CLI gates fail
# fast BEFORE any bus contact — the kt bounds in operator units (the
# config's [0.5, 1.0] scale bound expressed as effective torque
# constants), the REQUIRED exclusive log, and the reviewed duration
# bound shared with x7_float.
add_test(NAME gravity_demo_rejects_valueless_log
  COMMAND x7_gravity_demo --log)
set_tests_properties(gravity_demo_rejects_valueless_log PROPERTIES
  PASS_REGULAR_EXPRESSION "--log requires a value" LABELS unit)
add_test(NAME gravity_demo_rejects_flag_as_log_value
  COMMAND x7_gravity_demo --log --bogus)
set_tests_properties(gravity_demo_rejects_flag_as_log_value PROPERTIES
  PASS_REGULAR_EXPRESSION "--log requires a value" LABELS unit)
add_test(NAME gravity_demo_requires_log COMMAND x7_gravity_demo 5)
set_tests_properties(gravity_demo_requires_log PROPERTIES
  PASS_REGULAR_EXPRESSION "--log is REQUIRED"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_rejects_existing_log
  COMMAND x7_gravity_demo --log README.md 5
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(gravity_demo_rejects_existing_log PROPERTIES
  PASS_REGULAR_EXPRESSION "already exists"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_rejects_long_duration
  COMMAND x7_gravity_demo 61)
set_tests_properties(gravity_demo_rejects_long_duration PROPERTIES
  PASS_REGULAR_EXPRESSION "exceeds the reviewed"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_rejects_nonpositive_duration
  COMMAND x7_gravity_demo 0)
set_tests_properties(gravity_demo_rejects_nonpositive_duration PROPERTIES
  PASS_REGULAR_EXPRESSION "must be positive"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_rejects_kt_below_nominal
  COMMAND x7_gravity_demo --kt-xm430 1.5)
set_tests_properties(gravity_demo_rejects_kt_below_nominal PROPERTIES
  PASS_REGULAR_EXPRESSION "outside"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_rejects_kt_above_bound
  COMMAND x7_gravity_demo --kt-xm540 8)
set_tests_properties(gravity_demo_rejects_kt_above_bound PROPERTIES
  PASS_REGULAR_EXPRESSION "outside"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_rejects_valueless_kt
  COMMAND x7_gravity_demo --kt-xm430)
set_tests_properties(gravity_demo_rejects_valueless_kt PROPERTIES
  PASS_REGULAR_EXPRESSION "--kt-xm430 requires a value" LABELS unit)
add_test(NAME gravity_demo_rejects_malformed_kt
  COMMAND x7_gravity_demo --kt-xm430 nan)
set_tests_properties(gravity_demo_rejects_malformed_kt PROPERTIES
  PASS_REGULAR_EXPRESSION "invalid torque constant"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
# Reviewer directive (P1): the [0.5, 1.0] config bound is an
# electrical bound, not a validated float envelope — kt deviating
# from the approved vendor calibration requires the unmistakable
# --experimental-calibration opt-in, deviations warn in BOTH
# directions, and vendor-equal values pass without the flag. All
# before any bus contact.
add_test(NAME gravity_demo_rejects_experimental_kt_without_opt_in
  COMMAND x7_gravity_demo --kt-xm430 2.0)
set_tests_properties(gravity_demo_rejects_experimental_kt_without_opt_in
  PROPERTIES PASS_REGULAR_EXPRESSION "--experimental-calibration"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_accepts_vendor_kt_without_opt_in
  COMMAND x7_gravity_demo --kt-xm430 2.20 --kt-xm540 3.60)
set_tests_properties(gravity_demo_accepts_vendor_kt_without_opt_in
  PROPERTIES PASS_REGULAR_EXPRESSION "--log is REQUIRED"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_warns_hot_experimental_kt
  COMMAND x7_gravity_demo --experimental-calibration --kt-xm430 1.9)
set_tests_properties(gravity_demo_warns_hot_experimental_kt PROPERTIES
  PASS_REGULAR_EXPRESSION "HOTTER"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME gravity_demo_warns_weak_experimental_kt
  COMMAND x7_gravity_demo --experimental-calibration --kt-xm540 4.5)
set_tests_properties(gravity_demo_warns_weak_experimental_kt PROPERTIES
  PASS_REGULAR_EXPRESSION "WEAKER"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)

# x7_pose TCP targets: the CLI gates fail fast, and the preview path
# resolves IK and writes the rk_pen init file WITHOUT bus contact
# (a non-converged solve is a refusal, never a target).
add_test(NAME x7_pose_rejects_tcp_arity
  COMMAND x7_pose --tcp 0.2 0 0.3)
set_tests_properties(x7_pose_rejects_tcp_arity PROPERTIES
  PASS_REGULAR_EXPRESSION "requires 6 values"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_rejects_both_targets
  COMMAND x7_pose --posture config/postures/p1.json --tcp 0 0 0.5 0 0 0)
set_tests_properties(x7_pose_rejects_both_targets PROPERTIES
  PASS_REGULAR_EXPRESSION "exclusive"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_requires_a_target COMMAND x7_pose)
set_tests_properties(x7_pose_requires_a_target PROPERTIES
  PASS_REGULAR_EXPRESSION "a target is required"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_tcp_preview_writes_init
  COMMAND x7_pose --tcp 0.2 0 0.25 0 0 0
          --preview ${CMAKE_BINARY_DIR}/tcp_preview
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_pose_tcp_preview_writes_init PROPERTIES
  PASS_REGULAR_EXPRESSION "rk_pen"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_tcp_preview_refuses_unreachable
  COMMAND x7_pose --tcp 1.5 0 0.3 0 0 0
          --preview ${CMAKE_BINARY_DIR}/tcp_unreachable
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_pose_tcp_preview_refuses_unreachable PROPERTIES
  PASS_REGULAR_EXPRESSION "did not converge"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)

# Full TCP hardware path against the PTY emulator. Model loading and IK run
# after activation, so this catches a missing command keepalive: the emulated
# 100 ms Bus Watchdog would halt the move and make shutdown incomplete.
add_test(NAME x7_pose_tcp_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/x7_pose_tcp_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_pose>
          ${CMAKE_BINARY_DIR}/x7_pose_tcp_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_pose_tcp_smoke PROPERTIES
  LABELS integration TIMEOUT 25)

# End-to-end x7_gravity_demo over the pty emulator: startup and log
# contract with the default (vendor) constants, the custom-kt header
# derivation, and the ENTER early-stop.
add_test(NAME gravity_demo_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/gravity_demo_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_gravity_demo>
          ${CMAKE_BINARY_DIR}/gravity_demo_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(gravity_demo_smoke PROPERTIES LABELS integration)

# End-to-end log validation over the pty emulator: structure, event
# semantics, and produced rows (check_float_log.py).
add_test(NAME x7_float_log_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/float_log_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_float>
          ${CMAKE_BINARY_DIR}/float_log_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_float_log_smoke PROPERTIES LABELS integration)

# x7_ct_mass_error seed CLI contract: uint64 parsed exactly — bad
# seeds rejected, the maximum accepted, 2^53 vs 2^53+1 distinct.
add_test(NAME x7_ct_mass_error_cli
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/ct_mass_error_cli.sh
          $<TARGET_FILE:x7_ct_mass_error>
          ${CMAKE_BINARY_DIR}/ct_mass_error_cli
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_ct_mass_error_cli PROPERTIES LABELS integration)

# x7_wave --zvs: the commanded reference exports as a playable
# 9-coordinate sequence over the pty emulator.
add_test(NAME x7_wave_zvs_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/wave_zvs_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_wave>
          ${CMAKE_BINARY_DIR}/wave_zvs_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_wave_zvs_smoke PROPERTIES LABELS integration)

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
