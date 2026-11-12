# Learning trials on hardware (draft procedure)

!!! warning "Draft awaiting the owner's decision"
    This page is a draft procedure for the robot's owner to review and decide
    on. Learning trials on the physical arm are **not approved**, and **no
    learning trial has been run on the arm**. The procedure rests on the
    method's [simulation acceptance](../theory/learning-control.md#simulation-acceptance),
    which is necessary and never sufficient: a simulation pass does not show
    how the arm will respond. Every number marked *proposal* is for the owner
    to confirm, change or reject before the first sitting.

The [bring-up checklist](bringup.md) governs every powered step on this page,
its [safety section](bringup.md#safety-read-first) above all. This page does
not repeat that text, and nothing here relaxes it. The commands are those of
[learning between follow trials](../guide/usage.md#learning-between-follow-trials)
and [learnability analysis of follow logs](../guide/usage.md#learnability-analysis-of-follow-logs);
the method is described in the
[position-based learning control](../theory/learning-control.md) note.

## Purpose and scope

The demonstration: tracking of one taught reference improves by repetition
with iterative learning control (ILC) acting through the position command, in
plain position mode at 100 Hz, on the servo-side follower `x7_follow`.
Optionally, and only as a separate decision afterwards (stage H2):
feedback-error learning (FEL) on the same reference and the transfer of its
weights to a second taught reference.

Out of scope:

- any adaptation inside a run: every offset is computed offline between runs
  and stays fixed during one (the proper form of the
  [Slotine and Li law](../theory/learning-control.md#slotine-and-li-adaptive-control-in-position-form)
  stays in simulation);
- current mode and current-based position mode;
- control rates other than 100 Hz;
- lifting or widening any existing limit, gate, tolerance or timeout (the
  offset limits can only be lowered, and are not lowered here).

## What changes on the arm

Compared with a plain `x7_follow` run of the same reference (details in
[the follower with an offset](../guide/usage.md#the-follower-with-an-offset)):

- **The command** at tracking sample `i` is `q_d(t_i) + delta[i]`: the
  reference plus a learned offset bounded by 0.15 rad on every joint and
  sample, whose change is bounded by 0.5 rad/s. The follower only reads and
  checks the offset file, before the bus is opened (`--check` included): its
  grid and [reference signature](../guide/usage.md#the-reference-signature)
  must be this run's, every value and rate must be within the limits, and
  every command must lie inside the model's joint limits. It then prints
  `command offset accepted: method METHOD, trial K, max |delta| X rad, sha256 HASH`.
- **A new phase, `offset_ramp`,** runs after homing: the command moves from
  the last homing command (the home goal, home correction included) to
  `q_d(0) + delta[0]` at no more than 0.5 rad/s per joint, then holds there
  for `home.settle_time_s`. As in homing, no tracking gate is evaluated in it.
- **With a zero offset** (every H0 run, and trial 0 of every series) the ramp
  removes the home correction before tracking starts. The arm therefore
  settles to its uncorrected rest during the ramp, instead of dropping there
  in the first tracking cycles as the plain follower does. Expect a sag of the
  size the plain run showed: in the archived plain run of the taught
  reference, the mean tracking error of the elbow (canonical joint 3) was
  39.8 mrad below the reference (theory note
  [evidence](../theory/learning-control.md#evidence)).
- **The final hold** commands `q_d(T) + delta[N-1]`: the last offset sample
  stays applied until the operator supports the arm and presses Enter. Abort
  holds are unchanged: they hold the measured posture and carry no offset.
- **The tracking-error safety gates are unchanged** and are judged on the
  reference `q_d`, never on the command. As in the plain run, they are
  evaluated only during tracking: homing has its own strict tolerance, and
  neither the `offset_ramp` nor the final hold evaluates a tracking gate. A
  gate trip ends the run in an abort hold at the measured posture, which
  also waits for the operator's Enter. The log keeps its schema:
  `qrefN_rad` is the reference, `qcmdN_rad` the command sent, and
  `errorN_rad` stays `q - qref`.

Everything else is the plain follower: homing, the feedback-synchronized
cycle, the servo Bus Watchdog and the host deadman, stale-command rejection,
the operator's final Enter and the verified shutdown. Posture runs
(`--hold-current`, `--sweep-current`) take no offset; the combination is
refused before the bus is opened.

Within a run, nothing in the software stops an offset that is wrong but inside
its bounds, apart from the tracking gates. With the example configuration's
values they warn at 0.2 rad and abort at 0.5 rad sustained or 1.0 rad, far
above the errors expected here; the owner may tighten them for the series
(never widen them), as proposed under
[open decisions](#open-decisions-for-the-owner). The bounds limit what such an
offset can do: the command departs from the
validated reference by at most 0.15 rad per joint, and that departure changes
by at most 0.5 rad/s. During a run the operator and the power cutoff are the
protection; between runs, the stop rules below.

## Preconditions

All of these hold before the first sitting. The trials do not start
otherwise.

1. **The bring-up safety conditions hold** for every powered run: the
   actuator power cutoff within reach (the cutoff is the stop; `deactivate()`
   is not an emergency stop), the operator at the robot for every powered
   run, the workspace under the arm clear, and the arm supported from below
   whenever torque goes off, which happens at the end of every run.

2. **The software** is the reviewed commit, built, with its full test suite
   green on that build (`ctest --test-dir build --output-on-failure`, labels
   `unit` and `integration`), and the commit hash recorded in the series
   notes. Every bundle's `manifest.toml` records `git_commit` and
   `git_dirty`; they must be that hash and `false`.

3. **The emulator rehearsal of the hardware frontend with an offset passes
   on the reviewed commit.** The ctest `x7_follow_offset_emulator_smoke`
   (`tests/integration/follow_offset_emulator_smoke.sh`, label
   `integration`) runs `x7_follow` over the bus emulator: a run with the zero
   offset, an `ilc` update on that run's hardware-frontend log, a run with
   the resulting nonzero offset through `offset_ramp`, tracking and the final
   hold, and an over-limit offset refused before the bus is opened. The trials
   do not start without it. It does **not** show how the arm responds: the
   emulated servo has no dynamics (no gravity, no friction) and moves straight
   to each goal, so it rehearses the plumbing and the lineage, not the arm.
   It also ends its hold by `wait_time_s`, so the operator's Enter-ended hold
   that the series uses is not covered by any automated test; that hold is
   unchanged from the plain follower, which has ended every earlier hardware
   follow run that way. In addition,
   rehearse this page's H0 command lines once against `dxl_emu`
   (`./build/apps/dxl_emu --link /tmp/ttyDXL &`, then `--port /tmp/ttyDXL` on
   every `x7_follow` run) in a rehearsal directory of its own. Without
   `--port`, `x7_follow` opens the port of the hardware configuration, which
   is the real arm's.

4. **A working directory for the series**, new, outside the repository, with
   a unique file name for every trial. Run every command from the repository
   root of the reviewed build; all paths are under `$series`:

    ```sh
    run_stamp=$(date +%Y%m%d-%H%M%S)
    series="/path/outside/the/repository/learning-ilc-${run_stamp}"
    mkdir "$series"   # refuses an existing directory
    ```

    ```text
    learning-ilc-STAMP/
      follow.toml  learn.toml  reference.zvs
      sim/                          the simulated series
      params_S_*.toml               servo parameter dumps of sitting S
      offset_0.csv ... offset_5.csv
      report_1.toml ... report_5.toml
      zero_1/  zero_2/              H0 runs 1 and 2 (bundles)
      trial_0/ ... trial_5/         trial_0 is H0 run 3 (bundles)
      h0_repeat.json  anatomy_*.json
      notes.txt                     sitting notes
    ```

5. **The reference and the follow configuration are fixed for the whole
   series.** The reference is the taught trajectory chosen for the
   demonstration (theory note [evidence](../theory/learning-control.md#evidence)),
   copied into the series directory; its SHA-256 equals that of the reference
   in its archived position-mode baseline bundle. `follow.toml` is a copy of
   `config/follow_example.toml` with:
    - `model` and `hardware_config` pointing at the reviewed checkout's
      `models/crane_x7/crane_x7.ztk` and `config/crane_x7.toml` (input paths
      are relative to the TOML file that names them), `reference =
      "reference.zvs"`, and `motor_parameters` as in the baseline bundle;
    - `[control] rate_hz = 100.0` and `mode = "position"`;
    - `[reference_processing]`, all five keys, as in the baseline bundle's
      `follow.toml`, and `[home]`, `[safety]` and `[finalization]` likewise
      (`[finalization] wait_time_s = 0`: the hold ends at the operator's
      Enter). These stay unchanged within the series: the reference signature
      covers `[reference_processing]`, and the update filters the error over
      the tracking and final-hold rows together. `[safety]` may instead hold
      the tightened gates proposed under
      [open decisions](#open-decisions-for-the-owner), if the owner chooses
      them before the series; being in this file, they are then the same in
      the simulated series;
    - the servo keys of `config/follow_sim_servo_like.toml` in `[simulation]`,
      so that the same file drives the simulated series (`x7_follow` ignores
      `[simulation]`);
    - `[learning]` without `offset` (every run names its offset with
      `--offset`) and with both limits at their defaults.

6. **The learn configuration** `learn.toml` is a copy of
   `config/follow_learn_example.toml` with `[ilc] phi = 0.5`; every other key
   keeps the default, which is the value in the [settings](#settings) table.

7. **The servo parameters are dumped** before the series and again after it,
   in every sitting (see [every sitting](#every-sitting)). The offset is not
   bound to the servo gains: nothing checks that the servos are the ones the
   offset was learned with, and the dumps are how a change is detected.

8. **The same series has first been run in the servo-like simulation**, with
   the same reference, follow and learn configuration and number of trials,
   and its outputs kept (the testing ladder). It must complete all five
   updates without a refusal, meet the [H1](#stage-h1-the-ilc-series) stop
   rules S3, S5, S6 and S7, have its aggregate RMS fall at every trial (S4
   with no non-repeatable part: the simulation is deterministic) and meet the
   success criterion; otherwise the hardware series does not start. A learned
   offset does not transfer from this simulation to the arm: the hardware
   series starts again from the zero offset.

    ```sh
    sim="$series/sim"
    mkdir "$sim"
    ./build/apps/x7_follow_learn init --follow-config "$series/follow.toml" \
      --output "$sim/offset_0.csv"
    for k in 0 1 2 3 4 5; do
      ./build/apps/x7_follow_sim --config "$series/follow.toml" \
        --offset "$sim/offset_$k.csv" \
        --log "$sim/trial_$k.csv" --motion "$sim/trial_$k.zvs" || break
      if [ "$k" -lt 5 ]; then
        ./build/apps/x7_follow_learn ilc --config "$series/learn.toml" \
          --follow-config "$series/follow.toml" --log "$sim/trial_$k.csv" \
          --offset "$sim/offset_$k.csv" --output "$sim/offset_$((k + 1)).csv" \
          --report "$sim/report_$((k + 1)).toml" || break
      fi
    done
    uv run --project tools tools/follow_learnability.py anatomy \
      --json "$sim/anatomy.json" "$sim"/trial_[0-5].csv
    ```

9. **The archive destination** has been prepared by the owner (see
   [records](#records)).

### Every sitting

Each sitting opens and closes the same way, following the
[bring-up steps](bringup.md#steps); `S` names the sitting in file names.

1. Power on and connect, with the power cutoff within reach.

2. Scan (no torque): `./build/apps/dxl_inspect --port /dev/ttyUSB0 scan`;
   ids 2 to 9 respond.

3. Read streaming (no torque): `./build/apps/x7_read 20`; positions track,
   voltage about 12 V, temperatures sane. Note the temperatures.

4. Dump the parameters (torque off):
   `./build/apps/dxl_inspect --port /dev/ttyUSB0 dump-params "$series/params_S_before.toml"`.

5. First motion, wrist only: `./build/apps/x7_move_simple 6 0.3`; anything
   but `complete` ends the sitting.

6. After the sitting's first `x7_follow` run, and again after its last, dump
   the parameters (torque off, arm at rest) to `params_S_first.toml` and
   `params_S_after.toml`, and note the temperatures from `x7_read 20`.

`params_S_first.toml` and `params_S_after.toml` must be identical; a
difference stops the series until the owner has reviewed it. The `before`
dump may differ from them in gain and profile registers, since every
activation writes the operating mode (on which the servo resets its gains and
profiles), then the follower's position P gain and any motor-parameter file
([backing up and restoring motor parameters](../guide/usage.md#backing-up-and-restoring-motor-parameters)).
Record such a difference; a difference in any other field stops the series.

### Conducting one run

Every `x7_follow` run of a series is conducted the same way. The console
behaviour below is that of the follower's source, as seen in the emulator
rehearsal.

- **Before the bus is opened** the follower prints `command offset accepted:
  method METHOD, trial K, max |delta| X rad, sha256 HASH`: check the method,
  the trial and the hash against the offset meant for this run. It then
  prints `bus: ...` and activates.
- **During homing, the `offset_ramp` and tracking nothing is printed** unless
  a warning is raised (a `warning:` line, for example `warning: tracking error
  X rad on joint J`). A `warning:` line is recorded, and it fails the run
  under G1 and S1.
- **The `offset_ramp`** takes at most (0.15 + 0.15) / 0.5 = 0.6 s with the
  default limits (the home correction and the offset's first sample are each
  at most 0.15 rad per joint), rounded up to whole cycles, plus
  `home.settle_time_s` (0.3 s in the example configuration). With the zero
  offset a slow settle of tens of mrad, mostly at the elbow, is expected; a
  jump, vibration or noise is not (cut power, as the bring-up page says).
- **When tracking ends** the follower prints `tracking complete: support the
  arm from below, then press Enter to disable torque (timeout T s)`. Do not
  touch the arm for the first 2 s of the hold: the update reads up to the
  first 200 hold rows (2 s at 100 Hz) and filters them together with the
  tracking rows. Then support the arm from below and press Enter. An Enter
  typed before the hold begins is discarded (the hold's first check of the
  keyboard drains it). Without an Enter the run ends after
  `operator_timeout_s` (60 s in the example configuration) with status
  `operator_timeout` and a nonzero exit status, and torque then goes off as
  at any end of a run: the arm must be supported before that. An abort hold
  prints `run aborted: support the arm from below, ...` and waits for Enter
  in the same way.
- **At the end** the follower prints `created bundle ...`, the result line
  `follow hardware: STATUS, N controller cycles, home X rad, tracking Y rad`,
  then the `hardware timing:` and `hardware faults:` lines. In the result
  line, `home` is the worst home error and `tracking` the worst error of the
  measured position against the reference `q_d` over the tracking samples,
  not against the command.

## Settings

The ILC candidate from the simulation acceptance. None of these changes
during a sitting or a series: a different setting is a different series,
proposed and simulated first.

| Setting | Value | Source |
|---|---|---|
| `ilc.phi` | `0.5` | gain of the [simulation acceptance](../theory/learning-control.md#simulation-acceptance) candidate |
| `ilc.psi_s` | `0.0` | simulation acceptance |
| `ilc.advance_s` | `0.05` s | the measured servo lag, 39 to 77 ms on joints 0 to 5 (theory note [evidence](../theory/learning-control.md#evidence)) |
| `ilc.cutoff_hz` | `2.0` Hz | cut-off of the simulation acceptance candidate |
| `ilc.max_trial_step_rad` | `0.05` rad | simulation acceptance |
| `limits.max_offset_rad` and `[learning] max_offset_rad` | `0.15` rad | the follower's ceiling (also the home-correction clamp) |
| `limits.max_offset_rate_rad_s` and `[learning] max_offset_rate_rad_s` | `0.5` rad/s | the follower's ceiling; the `offset_ramp` moves at it |
| updates | five (trials 0 to 5) | simulation acceptance |
| `control.rate_hz`, `control.mode` | `100.0`, `position` | the only validated hardware rate; the mode of the study |
| `[reference_processing]`, `[finalization]` | as in the archived baseline run | precondition 5 |

The learn configuration's `[limits]` and the follow configuration's
`[learning]` limits are independent; keep them equal, as here, or the tool can
write an offset that the follower then refuses.

## Stage H0: repeatability with a zero offset

H0 is the first hardware step, and also the first run of the offset-capable
follower on the arm. It runs the zero offset three times.

1. Open the sitting ([every sitting](#every-sitting), steps 1 to 5).

2. Write the zero offset:

    ```sh
    ./build/apps/x7_follow_learn init --follow-config "$series/follow.toml" \
      --output "$series/offset_0.csv"
    ```

3. Check it without opening the bus:

    ```sh
    ./build/apps/x7_follow --config "$series/follow.toml" \
      --offset "$series/offset_0.csv" --check
    ```

    Expect `command offset accepted: method zero, trial 0, max |delta| 0 rad,
    sha256 ...` and then `configuration and reference preflight passed`.

4. Three runs with that same zero offset, one at a time, into the bundles
   `zero_1`, `zero_2` and `trial_0` (the third run is H1's trial 0):

    ```sh
    ./build/apps/x7_follow --config "$series/follow.toml" \
      --offset "$series/offset_0.csv" --bundle "$series/zero_1"
    ```

    Conduct each run as in [conducting one run](#conducting-one-run): watch
    the `offset_ramp`, keep hands off for the first 2 s of the final hold,
    then support the arm from below and press Enter, as the bring-up page
    describes.

5. After each run, before the next, record in the notes: the exit status (0
   only when the status is `success`, the arm was released and the log is
   complete), the result line `follow hardware: STATUS, ...`, the `hardware
   timing:` line (stale submissions, deadline misses) and the `hardware
   faults:` line (read and write failures, maximum feedback age), read as the
   bring-up [troubleshooting](bringup.md#troubleshooting) explains them.
   Inspect the result and let the arm and servos settle before the next run.
   After `zero_1`, take the `params_S_first.toml` dump.

6. After the three runs (the logs are each bundle's `hardware.csv`):

    ```sh
    uv run --project tools tools/follow_learnability.py repeat \
      --json "$series/h0_repeat.json" "$series/zero_1/hardware.csv" \
      "$series/zero_2/hardware.csv" "$series/trial_0/hardware.csv"
    uv run --project tools tools/follow_learnability.py anatomy \
      --json "$series/anatomy_zero_1.json" "$series/zero_1/hardware.csv"
    uv run --project tools tools/follow_learnability.py anatomy \
      --json "$series/anatomy_zero_2.json" "$series/zero_2/hardware.csv"
    uv run --project tools tools/follow_learnability.py anatomy \
      --json "$series/anatomy_0.json" "$series/trial_0/hardware.csv"
    ```

**Gate to continue to H1** (*proposal*, predeclared; every criterion must
hold):

| | Criterion | Read from |
|---|---|---|
| G1 | every run: status `success`, exit status 0, no `warning:` line, no stale submission, no deadline miss, no read or write failure | each run's console lines |
| G2 | non-repeatable error at most 3 mrad RMS over all 8 joints | `repeat`, row `all`, column `rms(r)` |
| G3 | non-repeatable error at most 20 % of the repeatable error | `repeat`, row `all`, `rms(r)` against `debiased` |
| G4 | initial-state spread within the homing tolerance, 0.01 rad | `repeat`, row `all`, column `q0 spread` |
| G5 | nothing unusual seen or heard | the operator |

A G1 failure is a failed run: stop the sitting and investigate as the bring-up
page says. If G2, G3 or G4 fails, stop: the study's premise, a repeatable
error, does not hold in position mode, and that is itself the result.

Even if nothing else is run, H0 delivers the first position-mode
repeatability measurement on this arm. The existing one is from
current-based position mode on another (planned) reference (theory note
[evidence](../theory/learning-control.md#evidence), provenance B).

## Stage H1: the ILC series

Trial 0 is the third H0 run (`trial_0`, zero offset): no extra run is needed,
and its log, `trial_0/hardware.csv`, with `offset_0.csv`, is what the first
update reads. Then, for `k` = 1 to 5, one trial at a time:

1. **Update** from the previous trial's log and offset:

    ```sh
    k=1   # the trial about to run; the update reads trial k - 1
    ./build/apps/x7_follow_learn ilc --config "$series/learn.toml" \
      --follow-config "$series/follow.toml" \
      --log "$series/trial_$((k - 1))/hardware.csv" \
      --offset "$series/offset_$((k - 1)).csv" \
      --output "$series/offset_$k.csv" --report "$series/report_$k.toml"
    ```

    Read the report before anything else: `measured_trial` is `k - 1` and
    `output_trial` is `k`; `tail_samples` is 200 (fewer means the hold was
    cut short and the update read a different record from the simulated
    one: stop); `[tracking] aggregate_rms_rad`; `[offset] max_abs_rad` and
    `max_rate_rad_s` per joint and `aggregate_max_rate_rad_s` (stop rule S6);
    `[ilc] step_clamped` and `abs_clamped` per joint (stop rule S7); and the
    `[[input]]` paths and hashes name the expected log and offset. The
    printed table shows the same per-joint values (columns
    `offset_max_rate_rad_s`, `step_clamped`, `abs_clamped`), and its last line
    is `offset max rate of trial K: X rad/s over all 8 joints`.

2. **Check** the new offset without opening the bus:

    ```sh
    ./build/apps/x7_follow --config "$series/follow.toml" \
      --offset "$series/offset_$k.csv" --check
    ```

    It must print `command offset accepted: method ilc, trial K, ...` with
    this `k` and the SHA-256 the update printed for `offset_$k.csv`.

3. **Run** it, conducted as in [conducting one run](#conducting-one-run):

    ```sh
    ./build/apps/x7_follow --config "$series/follow.toml" \
      --offset "$series/offset_$k.csv" --bundle "$series/trial_$k"
    ```

4. **Record** the console lines as in H0 step 5, the operator's observations,
   and the anatomy:

    ```sh
    uv run --project tools tools/follow_learnability.py anatomy \
      --json "$series/anatomy_$k.json" "$series/trial_$k/hardware.csv"
    ```

    Evaluate the stop rules before the next update.

In the rules, `RMS_k` is trial `k`'s aggregate tracking RMS over all 8 joints
and `HF_k` the aggregate RMS of its error above 3 Hz. `anatomy` prints both
on one line per log, `aggregate RMS e: 8 joints X mrad, arm 0-6 Y mrad; above
3 Hz: 8 joints Z mrad, arm 0-6 W mrad` (`RMS_k` is X, `HF_k` is Z), and
writes them to its JSON as `aggregate.all.e_rms_rad` and
`aggregate.all.e_rms_above_split_rad` (the arm-joint values under
`aggregate.arm`). The per-joint values above 3 Hz are its table's `>3Hz`
column and each joint's `e_rms_above_split_rad`. The report of the update
that reads trial `k` gives `RMS_k` again, as `aggregate_rms_rad`.

**Stop rules** (predeclared; the numbers are *proposals*). Each one ends the
series for that sitting: no retry, no changed setting. S6 and S7 are read
from the update's report, so they stop the series before the new offset is
run.

| | The series stops when | Read from |
|---|---|---|
| S1 | the run does not end in `success` with exit status 0, or it printed a `warning:` line | the run's console lines |
| S2 | any stale submission, deadline miss, or read or write failure (H0 required none, so any is beyond what H0 showed) | `hardware timing:`, `hardware faults:` |
| S3 | the tool refuses an update (a limit, an integrity check); the limits are never edited to get past a refusal | the update's exit status and message |
| S4 | `RMS_k` exceeds `RMS_(k-1)` by more than twice the non-repeatable RMS measured in H0 | `anatomy`; `h0_repeat.json` |
| S5 | `HF_k` exceeds 1.5 times `HF_0`, or on any single joint the above-3 Hz RMS exceeds twice trial 0's | `anatomy` |
| S6 | the new offset's maximum rate exceeds 0.45 rad/s | the report's `[offset] aggregate_max_rate_rad_s` |
| S7 | `abs_clamped` is nonzero on any joint in any update, or `step_clamped` is nonzero on any joint in any update after the first | the report's `[ilc]` table |
| S8 | the operator sees or hears anything the plain follower did not do (vibration, noise, a joint hunting) | the operator |

S5 exists because the simulation's over-aggressive case showed that the
aggregate RMS alone does not reveal growing high-frequency content (see the
[simulation acceptance](../theory/learning-control.md#simulation-acceptance)).
S6 is 90 % of the 0.5 rad/s rate limit; `aggregate_max_rate_rad_s` is the
largest of the per-joint maximum rates. In the simulation of this reference
the nominal setting settled at 0.33 rad/s, and at 0.40 rad/s with joint
friction on, so a lower threshold could stop a healthy series. S7 watches the
two clamps of the ILC update. A nonzero `abs_clamped` means the offset has
reached its 0.15 rad limit: in the simulation of this reference that happened
only with a servo half as stiff as assumed, and the error then stopped
falling. The trial-step clamp acted in the simulation only in the first
update, on 50 samples of the elbow; a nonzero `step_clamped` in the first
update is therefore expected and is recorded, not a stop. On S8 the bring-up
page's stop procedure applies at once: cut power.

**After a stop** the series ends. If the owner wishes, the last accepted
offset (that of the last trial no rule stopped) may be re-run once, into a
bundle of its own (`confirm_K`), to confirm that it is still good; it is not a
trial, and nothing is updated from it. After an S8 stop or a power cut,
nothing is re-run in that sitting. Analysis happens offline.

**Success criterion** (*proposal*, predeclared): no stop rule triggered;
`RMS_0 > RMS_1 > RMS_2 > RMS_3`; and `RMS_5` at most one third of `RMS_0`.

The expectation, stated honestly: on the archived data the repeatable error
was about 18 mrad against 1 mrad non-repeatable (provenance B), but in
current-based position mode on another reference. A clear improvement is
plausible; the floor on the arm, in position mode on this reference, is
unknown.

## Stage H2 (optional): feedback-error learning

H2 is a separate decision, taken only after H1 has been reviewed. It needs its
own sittings, its own learn configuration and its own simulated series (the
loop of precondition 8 with `fel` in place of `ilc`, as
[a learning loop in simulation](../guide/usage.md#a-learning-loop-in-simulation)
describes), and the same stop rules except S7: feedback-error learning has
no clamps (an offset outside the limits is refused, which is S3), and its
report has a `[fel]` table instead of `[ilc]`.

Settings: `learn-fel.toml` is a copy of `config/follow_learn_example.toml`
with

```toml
[fel]
step = "least-squares"
gain = 0.5

[fel.regressors]
smooth_sigma_s = 0.12
```

and every other key at its default. In the simulation of this reference the
default regressor smoothing (0.04 s) ran into the offset rate limit; 0.12 s
kept the offset's rate at 0.36 rad/s. Five updates, as in H1.

1. Trial 0 (*proposal*): a new zero-offset run at the start of the H2
   sitting, into `fel/trial_0`, so that S4 and S5 compare against the same
   sitting.

    ```sh
    mkdir "$series/fel"
    ./build/apps/x7_follow_learn init --follow-config "$series/follow.toml" \
      --output "$series/fel/offset_0.csv"
    ./build/apps/x7_follow_learn init-weights --config "$series/learn-fel.toml" \
      --output "$series/fel/weights_0.toml"
    ./build/apps/x7_follow --config "$series/follow.toml" \
      --offset "$series/fel/offset_0.csv" --check
    ./build/apps/x7_follow --config "$series/follow.toml" \
      --offset "$series/fel/offset_0.csv" --bundle "$series/fel/trial_0"
    ```

    The run is conducted as in [conducting one run](#conducting-one-run) and
    recorded as in H0 step 5, with its `anatomy` written to
    `fel/anatomy_0.json`.

2. For `k` = 1 to 5, the H1 cycle with this update in step 1 and `fel/` in
   every path (offsets, reports, the bundles `fel/trial_$k`, the `anatomy`
   files):

    ```sh
    ./build/apps/x7_follow_learn fel --config "$series/learn-fel.toml" \
      --follow-config "$series/follow.toml" \
      --log "$series/fel/trial_$((k - 1))/hardware.csv" \
      --offset "$series/fel/offset_$((k - 1)).csv" \
      --weights "$series/fel/weights_$((k - 1)).toml" \
      --output-weights "$series/fel/weights_$k.toml" \
      --output "$series/fel/offset_$k.csv" --report "$series/fel/report_$k.toml"
    ```

3. The transfer check, only if all five updates ran without a stop. The
   second taught reference gets `follow-b.toml` (as `follow.toml`, with its own
   reference and the `[reference_processing]` of its own archived baseline
   run). Simulate both of its runs first; then write its zero offset and
   apply the trained weights to it:

    ```sh
    ./build/apps/x7_follow_learn init --follow-config "$series/follow-b.toml" \
      --output "$series/fel/b_offset_zero.csv"
    ./build/apps/x7_follow_learn synth --config "$series/learn-fel.toml" \
      --weights "$series/fel/weights_5.toml" \
      --follow-config "$series/follow-b.toml" \
      --output "$series/fel/b_offset_synth.csv"
    ```

    `synth` writes no report; its printed line `offset max rate of trial K:
    X rad/s over all 8 joints` gives the synthesized offset's rate for S6.
    Check both offsets as in H1 step 2, run the zero offset (bundle
    `fel/b_zero`), then the synthesized one (`fel/b_synth`), and compare
    their aggregate tracking RMS with `anatomy`. S1, S2, S6 and S8 apply as
    in H1, and S5 with the zero-offset run of the second reference in place
    of trial 0.

The simulation flatters this scheme. On hardware logs recorded without an
offset, the same regressors predicted a held-out trajectory's error only to a
mean baseline-to-residual ratio of 2.47 (theory note
[evidence](../theory/learning-control.md#evidence)), an offline prediction,
not a closed-loop result. The gain to expect from H2, and from the transfer
in particular, is modest.

## Records

Kept for every trial: the bundle (recommended: `--bundle` binds the inputs,
the offset as `offset.csv`, the log as `hardware.csv`, `result.toml` with
`offset_method`, `offset_trial` and `offset_sha256`, and a SHA-256 manifest
with the commit), plus outside it the offset file the update wrote, the update
report and the `anatomy` JSON. Without `--bundle`, keep at least the log, the
offset, the report and the follow configuration. Kept for the series:
`follow.toml`, the learn configurations, `h0_repeat.json`,
the servo parameter dumps of every sitting, the simulated series directory,
the commit hash and its test result, and the sitting notes (date, operator,
console lines and exit statuses, observations, temperatures, every stop and
the rule that caused it).

Raw logs are never committed: the series directory is outside the repository.
After a sitting, the owner copies the whole series directory into the private
archive, and a section is added to the [data archive manifest](../records/data-archive.md)
following its conventions: one section per archived directory with its
archival date, one row per file with its role and SHA-256, and bundles listed
by their `manifest.toml`, as the existing follow sections do. Every hardware
bundle holds a file named `hardware.csv`, so records cite a run by its
directory-qualified name (for example `trial_3/hardware.csv`). The archive's
location is never written into a tracked file.

The learning curve reported is `RMS_0` to `RMS_5` (from the update reports and
`anatomy`), together with the H0 `repeat` result and each trial's `anatomy`
(per joint, and above 3 Hz). The theory note and the
[overview's evidence table](../guide/overview.md) are updated only from these
archived records; a simulation result is never quoted as hardware evidence.

## Open decisions for the owner

- Whether to run H0 at all. It stands on its own as a measurement.
- The H0 gate numbers (G2 to G4), the H1 stop-rule numbers (S4 to S7) and the
  success criterion. One point to settle: on a joint with almost no error
  above 3 Hz in trial 0, the per-joint part of S5 can trigger on noise alone.
- Whether to tighten the tracking gates for the series. The rules allow it;
  only widening is forbidden. The default is `[safety]` as in the baseline
  bundle. A *proposal*: `warning_error_rad = 0.15`,
  `sustained_abort_error_rad = 0.2` with `sustained_abort_time_s = 0.2`, and
  `immediate_abort_error_rad = 0.3`, against a worst tracking error of
  0.104 rad in the archived plain run of this reference. The gates are
  evaluated only during tracking, against the reference; a trip ends the run
  in an abort hold at the measured posture. These settings are not part of
  the reference signature, so they can be set without invalidating offsets,
  but they must be chosen before the series and then be the same in the
  simulated series (the shared `follow.toml` ensures it).
- The number of trials per sitting, and whether a series may be split across
  sittings or days. The offset is bound to the reference, its processing, the
  rate and the mode, not to the day; servo temperature is not controlled, and
  the servo gains are checked only through the dumps.
- The support arrangement during the runs, beyond the support the bring-up
  page requires whenever torque goes off.
- Whether H2 is wanted, and if so whether its trial 0 is a new zero-offset run
  (as proposed above) and what result of the transfer check counts.
