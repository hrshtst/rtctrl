# Control-rate qualification

rtctrl's hardware loops run at 100 Hz, the only rate validated on the arm.
This page is the procedure for measuring and qualifying higher rates (200, 500
and 1000 Hz), one operating mode at a time. It follows the approved plan
(revision 4, 2026-11-02). The 100 Hz default does not change here, and nothing
on this page changes a controller gain or the `x7_track` scale cap.

**Status.** The measurement tooling of Phase A exists: the torque-off bus
probe, the per-cycle timing log, the posture references and the timing
summary. E0 and E1 ran on 2026-11-03, and checkpoint 1 is decided (below).
E1 calibration is to be repeated with the corrected acknowledgement. Next
come the bounded bus transaction and the selected communication options
(Fast Sync Read and a motion-only read), then the rate-derived safety
options. No rate above 100 Hz is qualified, and every rate other than
100 Hz prints an experimental warning.

## What the numbers mean

Every timing quantity belongs to one of three classes, and reports keep them
apart:

- **Measured**: host-clock intervals between stamps the host took (wake
  lateness, the host transaction interval, the reply span, controller time,
  limiter and host write times, host work per cycle). These are exact for the
  recorded cycles. The reply span runs from the SDK's return of the first
  status packet to its return of the last: it cannot tell servo delays, USB
  buffering, driver delivery and SDK polling apart.
- **Estimated**: a statistic of calibration data applied to a run, such as a
  completion-offset quantile. It estimates a quantile; it bounds no individual
  cycle.
- **Envelope**: a check that relies on an assumed worst case, such as the
  maximum completion offset `delta_env`. The assumption is stated wherever the
  envelope is used.

The stamps are host observations. `read_start` is when the host began the
feedback transaction, not when the servos sampled: a register read does not
establish when its contents were measured, so the age of a servo's internal
measurement is unknown. `write_return` is when the host's `write()` returned,
not when the bytes left the adapter. The only per-write bound on wire
completion comes from a calibration run's acknowledged read (below).

## Session rules

- [Bring-up](bringup.md) applies throughout: the actuator power cutoff within
  reach, steps 1 and 2 at the start of the session, and the session's first
  motion is `x7_move_simple 6 0.3` (the wrist, at 100 Hz).
- Use a unique log name for every attempt, for example
  `rate-<step>-<mode>-<rate>hz-<YYYYMMDD>-<n>.csv`. The tools refuse an
  existing file.
- Raw logs go to the private archive with a manifest row, per the
  [data archive](../records/data-archive.md); they are never committed.
  Outcomes go to the [project history](../records/history.md).
- "Loaded" means `stress-ng --cpu $(nproc) --hdd 1 --timeout 75s` running
  alongside, with `--temp-path` on a disk (on tmpfs the `--hdd` worker loads
  memory, not a disk), or, if stress-ng is unavailable, a parallel clean
  rebuild of rtctrl in a separate build directory (never the `build/` the
  probe runs from). Record the exact command.
- Go 200, then 500, then 1000 Hz per mode, and attempt a rate only after
  every criterion passed at the previous one.
- Never position the arm at a rate that has not qualified. Powered steps at a
  new rate start from the measured posture (`--hold-current`,
  `--sweep-current`): such a run never homes or corrects, and aborts once a
  joint's fresh feedback is 0.05 rad from the reference it would be commanded
  (`posture_departure`, provisional), holding that sample's posture.
- **Stop rule.** Stop on any escalation or `DEADMAN`, any abort, oscillation,
  chatter, unexpected motion, or a failed criterion. Record it, do not retry
  that rate in the session, and change no limit before the analysis.

## Exploratory steps

Exploratory runs inform the checkpoint decisions. They are never acceptance
evidence.

### E0: inventory (torque off)

With the arm at rest and supported, and every servo torque-off:

```sh
./build/apps/x7_bus_timing --inventory --out "rate-e0-inventory-$(date +%Y%m%d).toml"
```

The inventory records, per servo, the firmware, Baud Rate register, Return
Delay Time, Drive Mode, Status Return Level, operating mode, torque state, Bus
Watchdog and hardware error status. It also records the tty's `latency_timer`,
the USB adapter's vendor and product IDs and USB speed, the kernel and the CPU
governors. The probe never writes Torque Enable or Drive Mode. It refuses to
run a timing measurement if any servo has torque on, or has Drive Mode bit 3
(Torque On by Goal Update, firmware v45 and later) set: with that bit, the
probe's goal writes would switch torque on. Activation refuses such a servo
too, before any write, so no powered app starts on it either. The inventory
prints a warning for such a servo; report it at checkpoint 1 before changing
any servo setting.

A Return Delay Time of 250 (500 us per reply, the factory value) adds about
4 ms to every eight-servo sync read, which by itself rules out rates above
about 200 Hz.

### E1: bus timing (torque off)

Production-traffic runs replay exactly the per-cycle traffic of one mode's
background thread (the grouped feedback read and the grouped goal write) at a
fixed rate. They are the only runs that can show a profile infeasible:

```sh
for rate in 100 200 500 1000; do
  for profile in current position cbp; do
    ./build/apps/x7_bus_timing --rate-hz "$rate" --profile "$profile" \
      --duration 60 --log "rate-e1-prod-$profile-${rate}hz-$(date +%Y%m%d)-1.csv"
  done
done
```

`--duration` is a deadline on the monotonic clock, not a cycle count: no cycle
starts after it, so a rate the bus cannot sustain runs fewer cycles rather
than longer, and a 60 s run stays inside the 75 s load window. The probe
checks the clock again after every wake, so a probe preempted or stopped in
its sleep past the deadline begins no further transaction. That wake has no
CSV row; the summary records it (`late_wake_stop`, `late_wake_lateness_s`)
and T5 counts its lateness. The summary also records the elapsed time
(`elapsed_s`).

Repeat the matrix under load, and with a real-time policy on a dedicated CPU
(`--rt-priority 80 --rt-cpu 3`, with an isolated core chosen for the machine).
The probe refuses a policy it cannot be granted. Stop a profile at its first
clearly failing rate.

Calibration runs add transport-timing observations and change the traffic, so
their cycle feasibility is never judged:

```sh
./build/apps/x7_bus_timing --rate-hz 200 --profile current --duration 60 \
  --confirm-writes --drain --log "rate-e1-cal-current-200hz-$(date +%Y%m%d)-1.csv"
```

`--confirm-writes` keeps the production write unchanged and follows every
write with a read of the last servo's Model Number, a read-only register.
With one writer on the line, the read request cannot go out before the write
(the SDK flushes received input, never pending output), and its response
cannot arrive before its own serialization and the servo's Return Delay
Time. A valid response (no error, the model number the inventory found)
therefore gives that write an upper bound on its wire completion,
`C+ = ack_return - ack_wire - return_delay`, whatever the servo did with the
goal. That matters: with torque off, firmware v47 in operating mode 0
overwrites goal writes with present values (observed 2026-11-03), which
defeated the earlier goal read-back. The summary counts invalid responses
(`ack_invalid`) and names the register read (`ack_register`). `--drain`
stamps `tcdrain()`, whose meaning is adapter-specific; it is reported beside
`C+`.

Each run writes a per-cycle CSV and a summary TOML (`<log>.summary.toml`) with
the run kind, the analytic wire budget, failure counts, the achieved frequency
and the phase histograms (bin upper edges, never below the recorded quantile).

`tools/e1_prod.sh` and `tools/e1_cal.sh` run E1 from the repo root, so no
command is typed by hand; `--dry-run` checks the prerequisites and prints the
plan. Before anything reaches the bus, each asks the operator to confirm that
every servo is torque-off, the arm is at rest and supported, and the power
cutoff is within reach. Before each run it waits for a quiet machine.
`e1_prod.sh` takes each condition (idle, loaded, rt-idle, rt-loaded), then
each profile, and climbs the rates, running `timing_summary.py --check` after
every run. When a check fails it asks whether that rate clearly failed: yes
ends that climb. A loaded run starts its own stress-ng window, which ends 15 s
after the run. With a real-time condition selected, the script refuses to
start while the shell's real-time priority limit (`ulimit -r`) is below the
priority; `sudo prlimit --pid $$ --rtprio=90:90` raises it for the shell that
starts the script. `e1_cal.sh` makes the calibration runs (current and
position at 100, 200, 500 and 1000 Hz, cbp at 100, 200 and 500 Hz) and prints
each envelope candidate. With `--rt-priority N --rt-cpu C` it runs them under
SCHED_FIFO pinned to that CPU, the checkpoint 1 baseline, with `rt-idle` in
the log names; each record states the scheduling requested and the policy
the probe was granted. In both scripts, a probe that exits nonzero or an
analysis that fails stops the session. Each run leaves
`<log>.csv.run.txt` beside its log and summary: the command, commit, load
command, scheduling, CPU governors, machine load, probe output and verdict.
Started again, a script skips every run already attempted (an interrupted run
counts) and keeps the climb decisions it recorded; `--attempt N` starts a new
series.

### Summaries

```sh
uv run --project tools tools/timing_summary.py rate-e1-prod-current-500hz-<date>-1.csv \
  --rate-hz 500 --check
```

`timing_summary.py` reads probe logs and app `--timing-log` files, prints
every quantity with its class, and with `--check` evaluates the criteria
below. Completeness (T3) and the controller criterion (T4) also read the run
summary written next to the log (`<log>.summary.toml`): whether every record
reached the file, the run's own verdict, and the controller counters at the
moment the controller finished, cross-checked against the per-cycle
`deadline_miss` flags. The shutdown windows after the controller finished are
reported, not counted. A criterion whose evidence is missing or inconsistent
(no summary, an unsuccessful run, no fresh command, counters the flags
contradict) is `INDETERMINATE`, never a pass, and `--check` exits 1 unless
every applicable criterion passes. Calibration logs also print the
completion-offset distribution and an envelope candidate (1.25 times its
maximum). The candidate comes only from a complete run with no read, write or
acknowledgement failure, since a failed cycle could have held the worst case;
otherwise the reason is printed instead. Logs from the earlier probe (summary
version 1, which compared goal read-backs) get no candidate either: each
matched row still bounds its own write, but the matches are a selected
subset, not a distribution.
`x7_controller_bench` measures the controller's own computation off-hardware
for comparison.

### Checkpoint 1

The reviewer receives the E0 inventory and the E1 summaries. From them they
select the communication changes (Phase B), set provisional envelopes, and
decide whether current-based position at 1 kHz is wanted (it needs 4 Mbps at
the 70% headroom rule).

**Decided 2026-11-03.** The legacy receive path is the main bottleneck:
SCHED_FIFO removes most of the scheduling jitter, but the read transaction
stays too slow for higher rates. At 500 Hz some real-time idle runs kept
their frequency but failed the headroom criterion, so 500 Hz is
insufficiently robust and 1 kHz remains an optimization target. Torque-off
measurements alone qualify neither 200 nor 500 Hz for powered control.

| Item | Decision |
|---|---|
| B1, Return Delay Time 0 | Already 0 on the arm: verify and record it; no EEPROM change |
| B2, motion-only read | Implement as a selectable profile with its auxiliary-read policy; budget the longest cycle |
| B3, Fast Sync Read | Proceed, on the strict-parser foundation (firmware v47 supports it) |
| B4, contiguous cbp goal | Conditional, until the first optimized measurements show its value |
| D8, cbp at 1 kHz | Target cbp at 500 Hz first; reconsider 4 Mbps after measuring the optimized 3 Mbps path |
| Scheduling | SCHED_FIFO with explicit CPU affinity is the comparison baseline. PREEMPT_RT is not the next priority. Affinity is pinning, not isolation |
| Provisional envelopes | Real-time wake 0.25 ms and limiter 40 us (maximum observed times 1.25, rounded up). Write completion (delta) and the separate USB and servo terms stay unassigned |
| Receive timeout | The new receive path starts with a conservative timeout on the legacy scale during torque-off validation, tightened from measurements of the implemented profile |
| Thresholds | The acceptance criteria stay unchanged |

Fast Sync Read with the 10-byte read saves about 0.24 ms of read
serialization. If the unexplained overhead stays, a 1.4 ms read still takes
about 1.16 ms, so the optimized path is measured before 1 kHz is promised.

**Calibration.** A valid response to the read that follows a write bounds
that write's completion even when the firmware overwrites the goal, so the
acknowledgement now reads Model Number after the unchanged production write
(see E1 above). The rows that matched in the first calibration runs are valid
per-write bounds but a selected subset: they give no distribution and no
envelope, and the mismatched rows cannot be recovered, because those runs
recorded transport success and value mismatch as one flag. The bound may
still be too loose for 1 kHz. External RS-485 capture is preferred where
available; otherwise separate diagnostic runs with USB tracing (`usbmon`) can
locate the batching delay. The archive manifests for E0 and E1 are added
before the next checkpoint.

## Later steps (after the bounded transaction and Phases B and C)

- **E2** repeats E1 for each implemented profile on the bounded transaction
  path.
- **E3** runs powered `x7_follow --hold-current` at candidate rates (no
  motion) to set write margins and check envelopes.
- **Checkpoint 2 (final freeze)** commits the thresholds and, per implemented
  profile, the envelopes measured on that profile, the margins per mode and
  rate, and the scheduling choice. Any later change voids the affected
  qualification runs.

Qualification runs are fresh runs with the frozen settings:

| Mode | Route | Steps per rate |
|---|---|---|
| position | `x7_follow` | Q1 `--hold-current 60`, Q2 `--sweep-current 6:0.3:6` (wrist), Q3 pick-approach (three repeats with a same-session 100 Hz baseline) |
| current-based position | `x7_follow` with the provisional configuration | Q1, Q2, Q3 as above |
| current | `x7_float` | Q4 the float acceptance protocol (three repeats with a same-session 100 Hz baseline) |

`x7_float` activates in position mode and switches to current mode
synchronously before its background thread starts, so its only periodic loop
is the current-mode one. Current-mode qualification therefore has no
position-mode prerequisite.

## Acceptance criteria (proposed; frozen at checkpoint 2)

| Criterion | Class | Requirement |
|---|---|---|
| T1 | measured | Mean period within 0.1% of nominal |
| T2 | measured | No skipped period idle; at most 1 per 10^4 cycles loaded, never two in a row |
| T3 | measured | No read or write failure, recovery, escalation, dropped timing record or log row, or envelope exceedance |
| T4 | measured | No stale rejection or controller deadline miss in a successful powered run |
| T5 | measured | Wake lateness p99.9 at most 0.10 T, maximum at most 0.5 T |
| T6a | measured | Host work per cycle p99.9 at most 0.70 T, maximum below T |
| T6b | envelope | `(bus_write_start - read_start) + delta_env` at most T in every cycle with a write |
| T7 | estimated | p99.9 of `(bus_write_start - read_start) + delta_q` at most 0.90 T |
| M1 | | No abort, escalation or observed oscillation |
| M2 | | Q3 joint RMS and elbow peak at most 1.10 times the same-session 100 Hz baseline |
| M3 | | Measured-torque power at 5 to 40 Hz at most 1.20 times the baseline |
| M4 | | `x7_float`'s objective float acceptance passes at each rate |

A threshold may change only before the freeze. Any change after it requires
fresh qualification runs.
