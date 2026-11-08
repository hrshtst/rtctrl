include(Catch)

add_executable(unit_tests
  unit/smoke_test.cpp
  unit/attitude_test.cpp
  unit/model_test.cpp
  unit/link_force_test.cpp
  unit/force_budget_test.cpp
  unit/force_budget_config_test.cpp
  unit/link_force_substep_test.cpp
  unit/link_force_log_test.cpp
  unit/async_writer_test.cpp
  unit/async_zvs_writer_test.cpp
  unit/realtime_test.cpp
  unit/clock_test.cpp
  unit/link_force_bundle_test.cpp
  unit/posture_test.cpp
  unit/trajectory_test.cpp
  unit/zvs_trajectory_test.cpp
  unit/ptp_planner_test.cpp
  unit/ptp_config_test.cpp
  unit/follow_config_test.cpp
  unit/follow_run_test.cpp
  unit/posture_reference_test.cpp
  unit/track_run_test.cpp
  unit/track_config_test.cpp
  unit/ik_test.cpp
  unit/dxl_test.cpp
  unit/wire_budget_test.cpp
  unit/status_parser_test.cpp
  unit/cycle_timing_test.cpp
  unit/cycle_record_test.cpp
  unit/timing_log_test.cpp
  unit/hw_test.cpp
  unit/hw_bounded_read_test.cpp
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
  unit/gravity_support_test.cpp
  unit/teach_config_test.cpp
  unit/teach_recording_test.cpp
  unit/bringup_test.cpp
  unit/dxl_parameters_test.cpp
  unit/x7_common_test.cpp)
target_link_libraries(unit_tests PRIVATE rtctrl::rtctrl Catch2::Catch2WithMain)
# track_common_test preserves the backed-up practical app orchestration
# (settle gate and latency verification) directly from apps.
target_include_directories(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/apps)
# Test code refers to models/ and config/ by repo-root-relative paths.
catch_discover_tests(unit_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS unit)

# Replaces the global operator new to count allocations, so it cannot
# share an executable with the other unit tests.
add_executable(alloc_free_tests unit/alloc_free_test.cpp)
target_link_libraries(alloc_free_tests PRIVATE rtctrl::rtctrl
                      Catch2::Catch2WithMain)
catch_discover_tests(alloc_free_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS unit)

add_test(NAME x7_plan_ptp_generates_example
  COMMAND x7_plan_ptp --config config/ptp_example.toml
          --output build/test_ptp_example.zvs
          --diagnostics build/test_ptp_example.csv --motion-time 0.2)
set_tests_properties(x7_plan_ptp_generates_example PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "wrote .* frames"
  LABELS unit)

add_test(NAME x7_plan_ptp_generates_pick_example
  COMMAND x7_plan_ptp --config config/ptp_pick_example.toml
          --output build/test_ptp_pick_example.zvs
          --diagnostics build/test_ptp_pick_example.csv)
set_tests_properties(x7_plan_ptp_generates_pick_example PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "IK warnings: 0"
  LABELS unit)

add_test(NAME x7_plan_ptp_bundle_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/ptp_bundle_smoke.sh
          $<TARGET_FILE:x7_plan_ptp>
          ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
          ${CMAKE_BINARY_DIR}/ptp_bundle_smoke)
set_tests_properties(x7_plan_ptp_bundle_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "PTP bundle creation and replay OK"
  LABELS integration)

add_test(NAME x7_force_budget_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/force_budget_smoke.sh
          $<TARGET_FILE:x7_plan_ptp>
          $<TARGET_FILE:x7_force_budget>
          ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
          ${CMAKE_SOURCE_DIR}/models/crane_x7/crane_x7.ztk
          ${CMAKE_BINARY_DIR}/force_budget_smoke)
set_tests_properties(x7_force_budget_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "force budget smoke OK"
  LABELS integration)

# The force-budget screen is offline: CLI errors reject before any model,
# reference, or bus work (no "bus:" line ever).
add_test(NAME x7_force_budget_requires_config COMMAND x7_force_budget)
set_tests_properties(x7_force_budget_requires_config PROPERTIES
  PASS_REGULAR_EXPRESSION "--config is required"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_force_budget_rejects_unknown_option
  COMMAND x7_force_budget --config config/force_budget_example.toml --port x)
set_tests_properties(x7_force_budget_rejects_unknown_option PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "unknown option: --port"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_force_budget_help COMMAND x7_force_budget --help)
set_tests_properties(x7_force_budget_help PROPERTIES
  PASS_REGULAR_EXPRESSION "usage: x7_force_budget --config FILE"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)

add_test(NAME x7_follow_bundle_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/follow_bundle_smoke.sh
          $<TARGET_FILE:x7_follow_sim>
          $<TARGET_FILE:x7_plan_ptp>
          ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
          ${CMAKE_SOURCE_DIR}/models/crane_x7/crane_x7.ztk
          ${CMAKE_SOURCE_DIR}/config/crane_x7.toml
          ${CMAKE_BINARY_DIR}/follow_bundle_smoke)
set_tests_properties(x7_follow_bundle_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "follow bundle creation and replay OK"
  LABELS integration)

add_test(NAME x7_follow_link_force_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/link_force_smoke.sh
          follow
          $<TARGET_FILE:x7_follow_sim>
          $<TARGET_FILE:x7_plan_ptp>
          ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
          ${CMAKE_SOURCE_DIR}/models/crane_x7/crane_x7.ztk
          ${CMAKE_SOURCE_DIR}/config/crane_x7.toml
          ${CMAKE_SOURCE_DIR}/config/link_force_example.toml
          ${CMAKE_BINARY_DIR}/follow_link_force_smoke)
set_tests_properties(x7_follow_link_force_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "link force smoke OK \\(follow\\)"
  LABELS integration)

add_test(NAME x7_track_link_force_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/link_force_smoke.sh
          track
          $<TARGET_FILE:x7_track_sim>
          $<TARGET_FILE:x7_plan_ptp>
          ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
          ${CMAKE_SOURCE_DIR}/models/crane_x7/crane_x7.ztk
          ${CMAKE_SOURCE_DIR}/config/crane_x7.toml
          ${CMAKE_SOURCE_DIR}/config/link_force_example.toml
          ${CMAKE_BINARY_DIR}/track_link_force_smoke)
set_tests_properties(x7_track_link_force_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "link force smoke OK \\(track\\)"
  LABELS integration)

add_test(NAME x7_track_bundle_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/track_bundle_smoke.sh
          $<TARGET_FILE:x7_track_sim>
          $<TARGET_FILE:x7_plan_ptp>
          ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
          ${CMAKE_SOURCE_DIR}/models/crane_x7/crane_x7.ztk
          ${CMAKE_SOURCE_DIR}/config/crane_x7.toml
          ${CMAKE_BINARY_DIR}/track_bundle_smoke)
set_tests_properties(x7_track_bundle_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "track bundle creation and replay OK"
  LABELS integration)

add_test(NAME x7_track_plot_smoke
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    MPLCONFIGDIR=${CMAKE_BINARY_DIR}/mpl-cache
    uv run --project tools
    bash ${CMAKE_SOURCE_DIR}/tests/integration/track_plot_smoke.sh
      ${CMAKE_SOURCE_DIR}/tests/integration/track_bundle_smoke.sh
      $<TARGET_FILE:x7_track_sim>
      $<TARGET_FILE:x7_plan_ptp>
      ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
      ${CMAKE_SOURCE_DIR}/models/crane_x7/crane_x7.ztk
      ${CMAKE_SOURCE_DIR}/config/crane_x7.toml
      ${CMAKE_SOURCE_DIR}/tools/plot/track_comparison.py
      ${CMAKE_BINARY_DIR}/track_plot_smoke)
set_tests_properties(x7_track_plot_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "track telemetry plotting OK"
  LABELS integration)

add_test(NAME x7_follow_plot_smoke
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    MPLCONFIGDIR=${CMAKE_BINARY_DIR}/mpl-cache
    uv run --project tools
    bash ${CMAKE_SOURCE_DIR}/tests/integration/follow_plot_smoke.sh
      ${CMAKE_SOURCE_DIR}/tests/integration/follow_bundle_smoke.sh
      $<TARGET_FILE:x7_follow_sim>
      $<TARGET_FILE:x7_plan_ptp>
      ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
      ${CMAKE_SOURCE_DIR}/models/crane_x7/crane_x7.ztk
      ${CMAKE_SOURCE_DIR}/config/crane_x7.toml
      ${CMAKE_SOURCE_DIR}/tools/plot/follow_tracking.py
      ${CMAKE_SOURCE_DIR}/tools/plot/follow_mode_comparison.py
      ${CMAKE_BINARY_DIR}/follow_plot_smoke)
set_tests_properties(x7_follow_plot_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "follow telemetry plotting OK"
  LABELS integration)

add_test(NAME x7_plan_ptp_plot_smoke
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    MPLCONFIGDIR=${CMAKE_BINARY_DIR}/mpl-cache
    uv run --project tools
    bash ${CMAKE_SOURCE_DIR}/tests/integration/ptp_plot_smoke.sh
      $<TARGET_FILE:x7_plan_ptp>
      ${CMAKE_SOURCE_DIR}/tools/plot/ptp_trajectory.py
      ${CMAKE_SOURCE_DIR}/config/ptp_example.toml
      ${CMAKE_BINARY_DIR}/ptp_plot_smoke)
set_tests_properties(x7_plan_ptp_plot_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "PTP diagnostics plotting OK"
  LABELS integration)

add_executable(integration_tests
  integration/sim_arm_test.cpp
  integration/link_force_sim_test.cpp
  integration/link_force_run_test.cpp
  integration/pty_test.cpp
  integration/thread_test.cpp
  integration/gravity_sim_test.cpp
  integration/tracking_sim_test.cpp
  integration/ident_sim_test.cpp
  integration/quiesce_pty_test.cpp
  integration/bounded_read_pty_test.cpp)
target_link_libraries(integration_tests PRIVATE rtctrl::rtctrl
  Catch2::Catch2WithMain)
# ident_sim_test drives the app-layer IdentRun and TwoMassArm fixture.
target_include_directories(integration_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/apps)
catch_discover_tests(integration_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS integration)

# The bounded read's deadline with a receiver paused across it. This
# executable defines read() (one armed call pauses), so it stays apart from
# the other integration tests.
add_executable(bounded_read_deadline_test
  integration/bounded_read_deadline_test.cpp)
target_link_libraries(bounded_read_deadline_test PRIVATE rtctrl::rtctrl
  Catch2::Catch2WithMain ${CMAKE_DL_LIBS})
catch_discover_tests(bounded_read_deadline_test
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

# Timing-summary reductions and criteria on synthetic probe and app logs.
add_test(NAME timing_summary_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_timing_summary.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(timing_summary_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "timing summary regressions OK"
  LABELS unit)

# Traced-run correlation: the association and alignment checks, each of
# their failures, and the receive path on a synthetic probe run and trace.
add_test(NAME trace_correlate_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_trace_correlate.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(trace_correlate_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "trace correlate regressions OK"
  LABELS unit)

# The timer mode's analysis (tools/timer_service.py) on synthetic events:
# the TSC anchors' matching and validated uncertainty, the classes of a
# late timer service, the exerciser's actions, the background load.
add_test(NAME timer_service_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_timer_service.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(timer_service_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "timer service regressions OK"
  LABELS unit)

# The read-only idle-state counter snapshots of every traced session run
# (tools/idle_counters.py) on a fake cpuidle tree.
add_test(NAME idle_counters_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_idle_counters.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(idle_counters_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "idle counters regressions OK"
  LABELS unit)

# The fair server's probes (tools/fair_server.py): their layout from an
# excerpt of the kernel's BTF, their definitions and filters, the waits
# after waking with their attribution, and the controls' judgement.
add_test(NAME fair_server_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_fair_server.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(fair_server_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "fair server regressions OK"
  LABELS unit)

# The fair server's controls tool (tools/fair_server_control.py) in its test
# modes, without a real-time priority: its phases, samples and supervisor.
add_test(NAME fair_server_control_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_fair_server_control.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(fair_server_control_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "fair server control regressions OK"
  LABELS unit)

# The observer placement (tools/observer_affinity.py) on processes of the
# test's own: the record, confinement, restoration and the snapshot check.
add_test(NAME observer_affinity_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_observer_affinity.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(observer_affinity_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "observer affinity regressions OK"
  LABELS unit)

# The timer mode's rehearsal exerciser: its rounds, rows and stop (no timing
# bound; it never touches the bus).
add_test(NAME timer_exerciser_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_timer_exerciser.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(timer_exerciser_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "timer exerciser regressions OK"
  LABELS unit)

# The adapter latency timer tool on a fake sysfs tree, its two device reads
# replaced: no real port is touched.
add_test(NAME latency_timer_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_latency_timer.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(latency_timer_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "latency timer regressions OK"
  LABELS unit)

# The latency hold diagnostic's sequence, outcomes, cleanup and exit
# statuses on a stand-in adapter, then its real serial port class on the
# emulator's pseudo-terminal (only the latency timer stands in there).
add_test(NAME latency_hold_regressions
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_latency_hold.py
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(latency_hold_regressions PROPERTIES
  PASS_REGULAR_EXPRESSION "latency hold regressions OK"
  LABELS unit)
add_test(NAME latency_hold_emulator
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    uv run --project tools python tools/test_latency_hold.py
      --emulator $<TARGET_FILE:dxl_emu>
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(latency_hold_emulator PROPERTIES
  PASS_REGULAR_EXPRESSION "latency hold emulator OK"
  LABELS integration)

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
  COMMAND x7_pose --posture config/postures/p1.toml --tcp 0 0 0.5 0 0 0)
set_tests_properties(x7_pose_rejects_both_targets PROPERTIES
  PASS_REGULAR_EXPRESSION "exclusive"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_requires_a_target COMMAND x7_pose)
set_tests_properties(x7_pose_requires_a_target PROPERTIES
  PASS_REGULAR_EXPRESSION "a target is required"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_rejects_unknown_mode
  COMMAND x7_pose --posture config/postures/p1.toml --mode current)
set_tests_properties(x7_pose_rejects_unknown_mode PROPERTIES
  PASS_REGULAR_EXPRESSION "position or current-based-position"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_rejects_position_effort_ceiling
  COMMAND x7_pose --posture config/postures/p1.toml
          --mode position --effort-limit-nm 2.5)
set_tests_properties(x7_pose_rejects_position_effort_ceiling PROPERTIES
  PASS_REGULAR_EXPRESSION "only valid with"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_rejects_excessive_cbp_effort_ceiling
  COMMAND x7_pose --posture config/postures/p1.toml
          --mode current-based-position --effort-limit-nm 4.1)
set_tests_properties(x7_pose_rejects_excessive_cbp_effort_ceiling PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "exceeds the configured limit"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_tcp_preview_writes_init
  COMMAND x7_pose --tcp 0.2 0 0.25 0 0 0
          --preview ${CMAKE_BINARY_DIR}/tcp_preview
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_pose_tcp_preview_writes_init PROPERTIES
  PASS_REGULAR_EXPRESSION "rk_pen"
  FAIL_REGULAR_EXPRESSION "bus:" LABELS unit)
add_test(NAME x7_pose_posture_preview_writes_init
  COMMAND x7_pose --posture config/postures/p1.toml
          --preview ${CMAKE_BINARY_DIR}/posture_preview
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(x7_pose_posture_preview_writes_init PROPERTIES
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

add_test(NAME x7_teach_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/teach_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_teach>
          $<TARGET_FILE:dxl_inspect>
          ${CMAKE_SOURCE_DIR}/config/teach_example.toml
          ${CMAKE_BINARY_DIR}/teach_smoke)
set_tests_properties(x7_teach_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "x7_teach emulator workflow OK"
  LABELS integration)

add_test(NAME x7_bus_timing_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/bus_timing_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_bus_timing>
          $<TARGET_FILE:dxl_inspect>
          ${CMAKE_BINARY_DIR}/bus_timing_smoke)
set_tests_properties(x7_bus_timing_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "bus timing probe OK"
  LABELS integration)

# The E1 session scripts (tools/e1_prod.sh, tools/e1_cal.sh) over the
# emulator; they call timing_summary.py through uv, so the build-local uv
# cache keeps the test hermetic.
add_test(NAME e1_session_scripts_smoke
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    bash ${CMAKE_SOURCE_DIR}/tests/integration/e1_scripts_smoke.sh
      $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_bus_timing>
      ${CMAKE_BINARY_DIR}/e1_scripts_smoke $<TARGET_FILE:dxl_inspect>)
set_tests_properties(e1_session_scripts_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "e1 session scripts OK"
  LABELS integration)

# The timer service diagnostics' TSC sampler: bracketed pairs on one CPU,
# a clean stop on SIGTERM, refused arguments.
if(TARGET tsc_mono_pairs)
  add_test(NAME tsc_mono_pairs_smoke
    COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/tsc_mono_pairs_smoke.sh
      $<TARGET_FILE:tsc_mono_pairs> ${CMAKE_BINARY_DIR}/tsc_mono_pairs_smoke)
  set_tests_properties(tsc_mono_pairs_smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "tsc_mono_pairs smoke OK"
    LABELS integration)
endif()

# The batch-workload check and the operator's reversible host quieting
# (tools/batch_workloads.sh, tools/quiet_host.sh), with stand-in processes
# under test-only names and a stand-in systemctl.
add_test(NAME quiet_host_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/quiet_host_smoke.sh ${CMAKE_BINARY_DIR}/quiet_host_smoke
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(quiet_host_smoke PROPERTIES
  PASS_REGULAR_EXPRESSION "quiet host smoke OK"
  LABELS integration)

# The traced torque-off session (tools/trace_session.sh) over the emulator,
# with stand-in tracers and no sudo.
add_test(NAME trace_session_script_smoke
  COMMAND ${CMAKE_COMMAND} -E env
    UV_CACHE_DIR=${CMAKE_BINARY_DIR}/uv-cache
    bash ${CMAKE_SOURCE_DIR}/tests/integration/trace_session_smoke.sh
      $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_bus_timing>
      ${CMAKE_BINARY_DIR}/trace_session_smoke $<TARGET_FILE:dxl_inspect>
      $<$<TARGET_EXISTS:tsc_mono_pairs>:$<TARGET_FILE:tsc_mono_pairs>>)
set_tests_properties(trace_session_script_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "trace session script OK"
  LABELS integration)

add_test(NAME x7_follow_timing_log_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/follow_timing_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_follow>
          $<TARGET_FILE:x7_plan_ptp> ${CMAKE_SOURCE_DIR}
          ${CMAKE_BINARY_DIR}/follow_timing_smoke)
set_tests_properties(x7_follow_timing_log_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "follow timing log OK"
  LABELS integration)

add_test(NAME x7_track_timing_log_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/track_timing_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_track>
          $<TARGET_FILE:x7_plan_ptp> ${CMAKE_SOURCE_DIR}
          ${CMAKE_BINARY_DIR}/track_timing_smoke)
set_tests_properties(x7_track_timing_log_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "track timing log OK"
  LABELS integration)

add_test(NAME x7_follow_posture_smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/follow_posture_smoke.sh
          $<TARGET_FILE:dxl_emu> $<TARGET_FILE:x7_follow>
          $<TARGET_FILE:x7_plan_ptp> ${CMAKE_SOURCE_DIR}
          ${CMAKE_BINARY_DIR}/follow_posture_smoke)
set_tests_properties(x7_follow_posture_smoke PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "follow posture references OK"
  LABELS integration)

# Functional only: the benchmark runs and reports every controller. Its
# timings describe the CI host and are never asserted.
add_test(NAME x7_controller_bench_runs
  COMMAND x7_controller_bench --iterations 200)
set_tests_properties(x7_controller_bench_runs PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION "computed_torque +n=200"
  LABELS integration)

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
