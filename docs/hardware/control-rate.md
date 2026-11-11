# Control-rate qualification

rtctrl's hardware loops run at 100 Hz, the only rate validated on the arm.
This page is the procedure for measuring and qualifying higher rates (200, 500
and 1000 Hz), one operating mode at a time. It follows the approved plan
(revision 4, 2026-11-02). The 100 Hz default does not change here, and nothing
on this page changes a controller gain or the `x7_track` scale cap.

**Status.** The measurement tooling of Phase A exists: the torque-off bus
probe, the per-cycle timing log, the posture references and the timing
summary. E0 and E1 ran on 2026-11-03, and checkpoint 1 is decided (below);
the E1 calibration was repeated with the corrected acknowledgement. Phase R
is complete: the strict status parser, the bounded read with its recovery,
the bus checks that activation makes before arming it, and the probe on the
bounded path. Its torque-off runs on 2026-11-04 closed it for development
(below); so far only the torque-off probe uses the bounded read. The
selected communication options, the motion-only read (B2) and Fast Sync
Read (B3), were measured torque-off on 2026-11-05 (below), and targeted USB
and scheduler diagnostics ran torque-off on 2026-11-06 (below); the
rate-derived safety options (Phase C) are still to come. No rate above
100 Hz is qualified,
and every rate other than 100 Hz prints an experimental warning.

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
The CSV's `*_ns` stamps count from the run's epoch. For correlation with
scheduler and USB traces, the summary also records, once per run, that
epoch on `CLOCK_MONOTONIC` (`monotonic_origin_ns`: a stamp's absolute time
is the origin plus the stamp) and the kernel thread ID of the probe thread
that runs the cycles (`worker_tid`, the ID scheduler traces give it); the
probe prints both too.

`tools/e1_prod.sh` and `tools/e1_cal.sh` run E1 from the repo root, so no
command is typed by hand; `--dry-run` checks the prerequisites and prints the
plan. Before anything reaches the bus, each asks the operator to confirm that
every servo is torque-off, the arm is at rest and supported, and the power
cutoff is within reach. Before each run it waits for a quiet machine.
`e1_prod.sh` takes each condition (idle, loaded, rt-idle, rt-loaded), then
each profile, and climbs the rates, running `timing_summary.py --check` after
every run. When a check fails it asks whether that rate clearly failed: yes
ends that climb. With `--no-climb` it asks nothing and runs every listed rate,
recording each verdict: more runs than stopping at the first clear failure,
but no judgement during the session. A loaded run starts its own stress-ng window, which ends 15 s
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

**The bounded read path (Phase R).** `x7_bus_timing --read-path bounded`
measures the grouped read through `dxl::Port`'s bounded receive instead of
the SDK's:
- the replies are checked strictly against a deadline;
- after a timeout or a malformed reply, a quiet guard holds every
  transmission and the next read is a recovery read.

Torque-off measurements keep the conservative limits that checkpoint 1
chose: a 35 ms budget (the legacy scale of the SDK's timeout for this
read) and an equal guard, the probe's defaults. Before its first write,
the probe checks the bus profile read-only and refuses on a mismatch:
- every servo's Return Delay Time at 0, Status Return Level at least 1,
  and the baud register;
- the adapter's latency timer at 1 ms, where the device exposes one.

The summary's `[bounded_read]` table holds the read's counters (reads,
timeouts, malformed replies, recoveries, guard waits), and
`timing_summary.py` prints them. `e1_prod.sh` and `e1_cal.sh` take
`--read-path bounded` and add `-bounded` to the log names. A powered
session's Bus Watchdog rule does not apply: the probe runs torque-off
and leaves the watchdog alone.

**The motion-only read (Phase B2).** `x7_bus_timing --read-path bounded
--feedback motion` reads the feedback window's first 10 bytes (current,
velocity, position) and the full 13 bytes on auxiliary cycles: the first,
then every `--aux-interval-ms` (default 100, rounded up to whole cycles;
0 never after the initial read), as a powered session's
`CraneX7::Options::feedback_read` does. Both windows are armed on the
bounded path, and each CSV row's `read_len` says which one the cycle
read. A full-read cycle is the longest, so `timing_summary.py` reports
the transaction and host work of each read length separately. The
summary's `[budget]` table budgets the longest cycle the run's cadence
contains (`read_len` names its read): the full read's whenever a cycle
reads the full window, the motion-only read's with `--aux-interval-ms 0`.
A run that mixes the two gives the motion-only cycle's wire time beside
it. The initial full read before the run (a Sync Read on the legacy path)
is startup traffic, outside the log, and the table budgets it separately
(`startup_read_*`). `e1_prod.sh` and `e1_cal.sh` take `--feedback motion`
(with `--read-path bounded`) and `--aux-interval-ms A`; their log names
gain `-motion`, or `-motion-aux<A>ms`.

**Fast Sync Read (Phase B3).** `x7_bus_timing --read-path bounded
--read-instruction fast` reads the feedback by Fast Sync Read (0x8A): one
status packet for all eight servos, which they do not byte-stuff, instead
of eight. It needs firmware v45 or later (the arm runs v47); the profile
check refuses older servos before the first write. The reply is checked
as strictly as a Sync Read's: the broadcast ID, the exact length, each
servo's ID in the requested order and its error byte, and the packet's
CRC at the end. ROBOTIS documents that only that last CRC is the
controller's to check; the ones after each servo's data serve the servos.
After a failure the recovery read is one byte longer per servo, as for
Sync Read. It combines with `--feedback motion`. The scripts take
`--read-instruction fast` (with `--read-path bounded`), and their log
names gain `-fast`. A powered session selects it with
`CraneX7::Options::feedback_read.instruction`.

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
| Provisional envelopes | Real-time wake 0.25 ms (raised to 0.75 ms on 2026-11-04, [below](#phase-r-on-hardware), and to 5.1 ms on 2026-11-05, [below](#phase-b2-and-b3-on-hardware)) and limiter 40 us (maximum observed times 1.25, rounded up). Write completion (delta) and the separate USB and servo terms stay unassigned |
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
locate the batching delay. No receive-only RS-485 tap is available (owner,
2026-11-03), so D7 is closed and the diagnostics use USB tracing. The archive
manifests for E0 and E1 are added before the next checkpoint.

### Phase R on hardware

**Torque-off runs, 2026-11-04.** The E1 runs were repeated on the bounded
read path with the 35 ms budget and an equal guard: 48 production-traffic
runs covering every condition, profile and rate (`e1_prod.sh --no-climb`),
and 11 calibration runs under SCHED_FIFO.
- **Transport.** 1,000,929 bounded reads had no timeout, malformed reply,
  recovery or guard wait. The recovery path was therefore never entered on
  hardware; its evidence remains the emulator fault tests.
- **Verdicts.** Pass or fail matched the legacy runs in all 33 paired runs.
  200 Hz is still the highest rate that passes every criterion, 500 Hz fails
  the headroom criterion (T6a) in every run, and 1000 Hz is infeasible on
  this read.
- **Calibration.** The completion bound stayed as loose as on the legacy
  path (median 1.91 ms, values about 1 ms apart).
- **Default scheduling under load.** Transactions were slower than in the
  legacy runs of the day before, but the same stress command also loaded the
  machine more. Separate-day runs cannot exclude an implementation effect, so
  the difference cannot yet be attributed. An interleaved legacy and bounded
  comparison would separate the two; it is optional and runs only if
  explaining the difference becomes necessary.

**Decided 2026-11-04.** The reviewer closed Phase R for development. The
hardware results establish compatibility in healthy operation, and the
unchanged read latency shows that replacing the SDK's receive logic alone
does not solve the throughput problem.

| Item | Decision |
|---|---|
| Phase B | B2 (motion-only read) and B3 (Fast Sync Read) proceed on the bounded path. Each is measured separately, including the longest cycle with an auxiliary read |
| Budget and guard | Stay at 35 ms with an equal guard during Phase B development: shortening them changes failure handling without speeding successful reads |
| 200 Hz budget | A T6a pass establishes host-side throughput only. In one real-time loaded cbp cycle (10096), the write started 4.818 ms after the read; the 102-byte write needs at least 0.340 ms at 3 Mbps, so its earliest completion (5.158 ms) is past the 5 ms period before any controller computation or USB output delay. The optimized path and the complete controller loop are measured before powered qualification is considered |
| Wake allowance | The provisional real-time wake allowance becomes 0.75 ms, an assumption for the present configuration. The 0.25 ms of checkpoint 1 was exceeded (0.575 ms for current at 100 Hz and 0.336 ms for position at 200 Hz, both real-time loaded), and 1.25 times the maximum gives 0.719 ms. It is remeasured for the optimized profiles (superseded on 2026-11-05, [below](#phase-b2-and-b3-on-hardware)) |
| Diagnostics | USB tracing (`usbmon`) is the next diagnostic for the batching hypothesis and the loose acknowledgement bound. USB events are correlated with the cycle timestamps, and USB completion stays distinct from completion of the RS-485 transmission |

**Interpretation.** Small wake lateness measures the delay before a cycle
starts; it does not exclude interruptions during the receive transaction.
The bounded receiver timestamps replies in batches, so a reply span near
zero shows compressed host observations, not when the replies arrived on
the bus. The spans of about 1 ms are host observations too.

The E0, E1 and bounded-path files are in the
[archive manifest](../records/data-archive.md).

### Phase B2 and B3 on hardware

**Torque-off runs, 2026-11-05.** At commit `630df10`, the Phase R matrix was
repeated for five variants of the bounded read, with the 35 ms budget and an
equal guard:
- the motion-only read, with an auxiliary read every 100 ms (`--feedback
  motion`) or none after the initial one (`--aux-interval-ms 0`);
- Fast Sync Read of the full window (`--read-instruction fast`);
- Fast Sync Read with each motion-only cadence.

Each variant had 48 production-traffic runs (`e1_prod.sh --no-climb`) and 11
calibration runs under SCHED_FIFO: 295 runs and 5,224,577 bounded reads.

- **Transport.** One read timed out (current, 1000 Hz, real-time loaded,
  Fast Sync Read with the motion-only read, cycle 6209). There was no
  malformed reply or servo error. The receive loop observed the expiry
  50.954 ms after the read started, past its 35 ms deadline, and had parsed
  no reply section: a scheduling delay can make the host observe the expiry
  late, and the late data were not accepted. The guard held (the write
  returned at 70.038 ms, after the computed guard expiry at 70.013 ms), and
  the next read, the recovery read, succeeded. These are host observations.
  The CSV's `bus_write_start` is stamped before the port waits on the guard,
  and a write's return after the guard does not establish when its bytes
  left, so the log supports correct recovery behavior but is no wire
  capture; the software guard tests remain the guard's evidence. Torque and
  the Bus Watchdog were off, so no powered watchdog recovery was tested.
- **Verdicts.** Passing runs over the five variants and three profiles, by
  the applicable probe criteria (T4 and T6b are not established by these
  runs):

  | Requested rate | Default, idle | Default, loaded | SCHED_FIFO, idle | SCHED_FIFO, loaded |
  |---|---|---|---|---|
  | 100 Hz | 15/15 | 0/15 | 15/15 | 15/15 |
  | 200 Hz | 11/15 | 0/15 | 15/15 | 14/15 |
  | 500 Hz | 0/15 | 0/15 | 0/15 | 0/15 |
  | 1000 Hz | 0/15 | 0/15 | 0/15 | 0/15 |

  200 Hz is a candidate for selected real-time profiles, not a pass across
  variants or scheduling conditions: Fast Sync Read of the full window with
  current commands failed T6a under real-time load (host work p99.9
  3.670 ms against 3.500 ms). Every 500 Hz run failed T6a; the best
  real-time idle host work p99.9 was 1.696 ms against 1.400 ms (position,
  Fast Sync Read, motion-only read without auxiliary reads). Every 1000 Hz
  run failed T1, T2 and T6a. Its best mean rates, 695.6 Hz for current and
  676.5 Hz for position, are throughputs with skipped cycles, not 1 kHz
  loops; cbp stayed at 500 Hz.
- **Completion within the period.** A host-work pass is not completion
  qualification. In the failing run above, the write of cycle 4394 started
  4.917 ms after its read, and the 38-byte write needs at least 0.127 ms at
  3 Mbps: its earliest completion (5.044 ms) is past the 5 ms period. The
  motion-only Sync Read has another (cycle 10305 of the current, 200 Hz,
  real-time loaded run: 5.049 ms).
- **Read time.** The improvement matches the wire bytes saved. Unstuffed, a
  13-byte Sync Read takes 0.713 ms and a 10-byte Fast Sync Read 0.473 ms, a
  0.240 ms reduction; the median host transaction fell by about 0.22 to
  0.30 ms. (The analytic budget's 0.820 ms for the Sync Read includes
  worst-case stuffing.) Transactions cluster near whole milliseconds (about
  0.95, 1.95, 2.95 and 3.94 ms). That is evidence for investigating USB and
  adapter delivery; it does not identify the adapter's latency timer as the
  cause, nor exclude delayed host processing or servo-side gaps. Where the
  two reads alternate, full-read cycles took about 0.1 ms longer at the
  median than motion-only ones.
- **Calibration.** The acknowledgement bound stays loose: the completion
  median was 1.907 to 1.928 ms in 54 of the 55 calibration runs and
  2.894 ms in the other (cbp, 100 Hz, Fast Sync Read with the motion-only
  read), with no invalid acknowledgement. There is still no defensible
  sub-millisecond write-completion envelope.
- **Wake lateness.** Six real-time runs exceeded the 0.75 ms allowance:
  five loaded (0.798, 0.825, 0.935, 0.950 and 0.970 ms) and one idle
  (4.042 ms: cbp, 1000 Hz, Fast Sync Read of the full window, in a cluster
  of cycles 6.1 to 8.4 s into the run, cause unknown).

**Scheduling.** The kernel (7.0.0-42-generic) is built without
`CONFIG_RT_GROUP_SCHED`, so the real-time runtime settings
(`sched_rt_runtime_us` 950000 of 1000000) do not by themselves show the
classic real-time throttler at work. The fair deadline server, which
upstream Linux 7.0 initializes to 50 ms per 1000 ms, is a candidate for the
50 ms stall. Its settings, read on 2026-11-06 (below), are those defaults,
and its involvement in this event is a hypothesis. CPU 3 shares a physical
core with CPU 2, and the
competing load ran on both, so pinning the probe did not give it an
exclusive core.

**Decided 2026-11-05.** The reviewer accepted B2 and B3 as completed
exploratory measurements.

| Item | Decision |
|---|---|
| Rates | 200 Hz is a candidate for selected real-time profiles. 500 and 1000 Hz, and powered qualification of any rate above 100 Hz, stay open |
| Wake allowance | 5.1 ms (1.25 times the 4.042 ms maximum, rounded up): one provisional allowance for these unchanged real-time profiles, superseding 0.75 ms. It is an engineering assumption, not a guarantee or a T5 threshold, and qualifies no rate; it does not bound pauses within a read. A narrower one needs a narrower configuration, or evidence explaining and removing the outlier, then new measurements. It is remeasured after the scheduling environment is addressed, before checkpoint 2 |
| Budget and guard | Unchanged. The 35 ms receive budget constrains acceptance, not how soon the host reacts when its thread stops running, and these observations do not justify shortening the budget or the guard |
| Scheduler | Record the fair server's CPU 3 `runtime` and `period` (`/sys/kernel/debug/sched/fair_server/cpu3/`) and capture scheduler events before changing any setting; a settings change is not an established fix |
| Next diagnostics | A few targeted torque-off runs, not the full matrix. usbmon transmit and receive events, scheduler events and the probe's timestamps are correlated through the probe worker's thread ID and a once-per-session absolute `CLOCK_MONOTONIC` origin for the CSV, whose semantics stay unchanged. The runs cover Fast Sync Read with the motion-only read at 500 and 1000 Hz, quiet and loaded, with and without CPUs 2 and 3 excluded from the competing load (load stays on the other CPUs), and the cbp 1000 Hz idle anomaly. They record the fair-server settings, the topology, affinities and IRQ placement, and include comparable untraced runs for the tracing overhead. They locate each delay: before USB completion, between completions, or after the data reached the host but before the worker consumed it. usbmon records the driver and host-controller interface, not the last RS-485 byte |
| Powered work | Both project watchdogs stay enabled, and T5 and T6a stay |

**Scheduler settings (read 2026-11-06).** Secure Boot is enabled, so the
kernel runs in integrity lockdown (`/sys/kernel/security/lockdown`), which
refuses access, even to root, to debugfs files that can be written. The fair
server's `runtime` and `period` files under
`/sys/kernel/debug/sched/fair_server/` are such files, so they were read
instead from the kernel's live scheduler structures, with a read-only
bpftrace probe on `dl_server_start` and `dl_server_update`. On every CPU (0
to 31), CPUs 2 and 3 included, the fair server has a 50 ms runtime, a
1000 ms period and relative deadline, and is deferred (`dl_defer`); the
probe saw no sched_ext server. These are the upstream defaults. The lockdown
refuses writes to those files too, so the settings have not changed since
boot and cannot be changed without lifting it. A 50 ms runtime is consistent
with the stall's length but does not establish the fair server's
involvement, which stays a hypothesis until scheduler events are captured.
Also recorded: `sched_rt_runtime_us` 950000 and `sched_rt_period_us`
1000000, and a boot command line without `isolcpus`, `nohz_full` or a
lockdown setting. The lockdown did not block the BPF probe, and it restricts
tracefs and perf only at the confidentiality level, which is not set.

The files are in `b2b3_bounded/` of the
[archive manifest](../records/data-archive.md).

### Traced torque-off diagnostics

`tools/trace_session.sh` runs the targeted diagnostics decided on
2026-11-05, torque-off, from the repo root. Like the E1 scripts, it asks the
operator to confirm the session, waits for a quiet machine before each run,
refuses real-time conditions below the shell's real-time priority limit, and
prints its plan with `--dry-run`. Its default runs, all on the bounded read
path under SCHED_FIFO 80 on CPU 3:
- Fast Sync Read with the motion-only read (the full window every 100 ms)
  for current at 500 and 1000 Hz, under rt-idle, rt-loaded and
  rt-loaded-excl;
- cbp at 1000 Hz with Fast Sync Read of the full window, under rt-idle.

An `-excl` run pins stress-ng to the CPUs outside the excluded set (CPU 3
and its SMT sibling, CPU 2, by default), with one CPU worker per CPU there.
`--runs` lists other runs as `profile:rate:condition:variant`.

Each run is made untraced and then traced under the same conditions, so
each pair shows the tracing overhead. A traced run records:
- scheduler, interrupt, timer, workqueue and xHCI URB events, with
  `trace-cmd record -C mono`. The mono clock is `CLOCK_MONOTONIC`, the clock
  of the probe's `monotonic_origin_ns`; ftrace's default `local` clock is not
  assumed equivalent. The script checks the trace clock while recording and
  stops the session before the probe runs if it is not mono;
- the adapter's usbmon traffic, with tcpdump limited to its USB device.
  usbmon stamps its events on `CLOCK_REALTIME` in microseconds, so the record
  also takes the tightest of 2000 `CLOCK_MONOTONIC`, `CLOCK_REALTIME`,
  `CLOCK_MONOTONIC` reads before and after the run: their offset maps the
  capture onto the probe's clock when the two agree (no clock step). The
  xHCI events in the trace give an independent alignment. usbmon records the
  driver and host-controller interface, not the last RS-485 byte.

The tracers run on the CPUs outside the excluded set. Before the first run,
the session reads the fair server's settings with the read-only bpftrace
probe above, and records them with the lockdown state, boot line, real-time
settings, topology and the adapter. Every run record repeats them, with the
run's IRQ placement before and after, and a snapshot halfway through each
run records every task's CPU, policy and allowed CPUs. Right before and
after the probe, every run, untraced ones included, also takes read-only
snapshots of the excluded CPUs' idle-state counters
(`tools/idle_counters.py`): each state's identity, disable flag and raw
counters with the monotonic times of the reads, and their deltas. A counter
the kernel does not offer is marked unavailable, and a state whose identity
changed gets no delta (reviewer decision, 2026-11-08). The session changes
no scheduler setting; it loads the usbmon module if needed and unloads it at
the end. A loaded run starts its load once the recorders are recording, so
their startup takes none of the load's time. The load and the recorders
must last the whole run: one that ends before the probe does voids the run,
even with a successful exit, and so does a load that does not report
success. A failed criterion is data. A probe that exits nonzero, a tracer
that does not start, stop cleanly or last the run, a load that does not
last the run or succeed, or an analysis error stops the session.

Each run leaves `<log>.csv.run.txt` and `<log>.csv.tasks.txt` beside its log
and summary, and a traced run also `<log>.csv.trace.dat` and
`<log>.csv.usbmon.pcap`, with `<log>` =
`rate-trace-<profile>-<rate>hz-<condition>-bounded<tags>-<untraced|traced>-<date>-<attempt>`.
The privileged steps go through sudo; the probe runs as the invoking user,
and the session gives the trace and the capture to that user on every path,
an interrupted session (Ctrl-C or SIGTERM) included. Before the arm, a
rehearsal over `dxl_emu` checks the tracers and the clock correlation:
`--port` the emulator's link (with Return Delay Time 0 written to the
emulated servos) and `--usb-device` the adapter's sysfs directory, whose
capture stays empty while nothing opens its port.

`uv run --project tools tools/trace_correlate.py <log>.csv` analyses a
traced run. It first checks association and alignment, and exits nonzero if
any check fails:
- each wake of the worker by its sleep timer (a waking inside that timer's
  callback) belongs, one to one, to the probe's sleep that contains it (from
  the previous cycle's end to the wake stamp); the expiry's trace time, the
  timer's own `CLOCK_MONOTONIC` time, the cycle's scheduled wake, the
  worker's switch-in and the probe's wake stamp must fall in causal order.
  A cycle without a wake must have begun without sleeping: its scheduled
  wake had passed, and the worker never left the CPU;
- the adapter's bulk OUT submissions pair one to one, in order, with the
  probe's read instructions and writes, each between the probe's stamps
  around its call;
- each Fast Sync Read's bulk IN completions add up to its reply (the
  adapter's two status bytes per packet removed), and no data arrives
  outside a read.

These checks show consistency, not a calibrated accuracy bound. The tool
then follows each reply from the adapter's last USB completion through the
USB completion work, the TTY work that moves the bytes to the line
discipline (`flush_to_ldisc`, matched by the port's work item and followed
per kernel worker through its end) and the worker's resumption. When the
completion work queued no TTY work (it was already pending), the TTY work's
queue time is not in the trace, and that read's wait is reported apart,
never as a queue wait. The tool locates each read's time before the first
USB completion, between completions or after the last one; for each
overrun it reports the largest measured of these parts, the wake lateness
and the write, which locates the time but does not identify its cause. It
also counts the adapter's bulk IN completions within the run and their
spacing.

**Torque-off runs, 2026-11-06.** At commit `b593b4d`, `tools/trace_session.sh`
ran its default plan: 14 runs of 60 s, seven conditions each untraced and
then traced. Every run completed without a read or write failure, timeout,
malformed reply or ring drop, and every recorder and load lasted its run
and exited 0. Every traced run recorded on the mono clock with no overrun or
dropped event on any of the 32 CPUs and no usbmon packet dropped, and its
clock pairs moved by at most 6 ns. Every run failed T2 and T6a, and all but
the two 500 Hz idle runs failed T1. `tools/trace_correlate.py` passed on all
seven traced runs. In the three 1000 Hz runs, 3, 1 and 2 cycles began
without sleeping, their scheduled wake 0.05 to 0.23 us after the previous
cycle's end stamp.

| Traced run | Replies in two completions | Before the first completion, p50 | After the last completion, p50 / p99 / max | TTY work's queue wait, p50 / p99 / max | Reads without a TTY queue event | Host transaction, p50 / p99.9 / max |
|---|---|---|---|---|---|---|
| current, 500 Hz, rt-idle | 52% | 636 us | 7.8 us / 137 us / 2.03 ms | 2.0 us / 107 us / 2.02 ms | 23 | 1.187 / 1.793 / 3.560 ms |
| current, 500 Hz, rt-loaded | 45% | 707 us | 318 us / 1.01 ms / 3.08 ms | 307 us / 991 us / 3.07 ms | 106 | 1.956 / 3.960 / 4.006 ms |
| current, 500 Hz, rt-loaded-excl | 51% | 649 us | 10.2 us / 593 us / 2.89 ms | 2.2 us / 566 us / 2.88 ms | 24 | 1.192 / 2.826 / 4.344 ms |
| current, 1000 Hz, rt-idle | 37% | 726 us | 7.0 us / 70 us / 168 us | 1.5 us / 63 us / 161 us | 1 | 0.994 / 1.714 / 1.796 ms |
| current, 1000 Hz, rt-loaded | 47% | 719 us | 114 us / 995 us / 3.83 ms | 104 us / 969 us / 3.82 ms | 80 | 1.366 / 3.948 / 5.170 ms |
| current, 1000 Hz, rt-loaded-excl | 37% | 728 us | 10.3 us / 718 us / 2.90 ms | 2.2 us / 707 us / 2.89 ms | 43 | 1.013 / 3.084 / 4.432 ms |
| cbp, 1000 Hz, rt-idle | 81% | 655 us | 7.1 us / 72 us / 169 us | 1.5 us / 65 us / 163 us | 2 | 1.495 / 1.989 / 2.062 ms |

The queue wait is measured on the reads whose USB completion work queued
the TTY work. For the others the work was already pending, its queue time
is not in the trace, and the wait from the completion work's start is
reported apart (up to 2.6 ms under load); the after-last-completion column
includes every read.

- **Before the last USB completion.** The adapter's bulk IN completions,
  with data or with only its two status bytes, came at a nominal 1 ms
  cadence: 60,002 within each traced run's 59.999 s, consecutive ones 999
  to 1000 us apart at the median. As the host observed them, their spacing
  ranged from 744 to 1203 us in the idle and `-excl` runs, and more widely
  under the full load (218 to 1817 us). The cadence matches the configured
  latency timer (1 ms). A reply that straddled a completion arrived in two,
  998 to 999 us apart at the median, which supports the replies waiting in
  the adapter's buffer for its next delivery; the trace and usbmon see the
  host side only, so the RS-485 arrival times are not measured. The time
  before the first completion includes the instruction's transport, the
  wire, the servos' turnaround, that buffering and the host's USB
  processing. The latency timer is a setting, and whether the adapter
  delivers sooner with a lower one is untested: these runs do not establish
  a 1 kHz limit of the hardware. At 1000 Hz the time between completions
  was the largest measured part of 70 to 74% of the overruns, and the time
  before the first of 26 to 28%; 29,985 of the cbp run's 30,001 cycles
  overran, so it ran at 500 Hz.
- **After the last USB completion.** Idle, the host delivered the data to
  the worker within 7.0 to 7.8 us at the median. Each read's last step waits
  for the TTY work on a normal-priority kernel worker that may run on any
  CPU: in 93% or more of each run's reads the worker was blocked in that
  work's flush when the work ended, and was woken by that kernel worker.
  Under the full load the TTY work's queue wait was 307 us (500 Hz) and
  104 us (1000 Hz) at the median, up to 3.8 ms; in the slowest reads its
  kernel worker was switched in after stress-ng and other tasks. Keeping the
  load off CPUs 2 and 3 restored the median (2.2 us) but not the tail (p99
  566 and 707 us, up to 2.9 ms), and some of the slowest reads' TTY work ran
  on CPUs 2 and 3. At 500 Hz under the full load, this time was the largest
  measured part of 205 of the 450 overruns, the time between completions of
  183 and the time before the first of 62; under the `-excl` load, the time
  between completions of 238 of 300 and this time of 62.
- **Tracing overhead.** The tracers run on the CPUs outside 2 and 3, among
  the load. Idle, a traced run's host transaction matched its untraced
  pair's within 0.06 ms at p50 and p99.9. Under the full load the traced
  median was higher (500 Hz: 1.956 against 1.567 ms; 1000 Hz: 1.366
  against 1.215 ms); under either load the p99.9 stayed within 0.15 ms of
  the untraced pair's, and the 500 Hz `-excl` run skipped 303 periods traced
  against 151 untraced. With one pair per condition, these differences
  suggest that the tracing perturbs a loaded run; they do not isolate its
  cost.
- **Wake lateness.** The one T5 failure (traced, 1000 Hz, full load: max
  1.427 ms) comes from 15 cycles, 0.53 to 1.43 ms late, within 1.2 s (42.2
  to 43.4 s into the run). In each, the worker reached its sleep 0.5 to
  1.6 ms after its cycle-end stamp, which precedes the cycle's bookkeeping
  and the sleep call, and its timer then expired on time from there. It was
  not descheduled in between; that does not show that the probe's code ran
  throughout (interrupts or time in the kernel remain possible), and the
  recorded events do not show what it did. Its untraced pair's wake
  lateness stayed within 0.034 ms. The trace does not implicate the fair
  server or the tracing itself.
- **The 50 ms stall** of 2026-11-05 did not recur: the longest host
  transaction was 5.170 ms. The fair server's involvement in it stays a
  hypothesis.

These are exploratory torque-off measurements. They change no setting and
qualify no rate; the receive budget, the guard and the wake allowance stay
as decided.

### Adapter latency timer experiment (torque-off)

The traced runs located most of a quiet read's time before the host has the
data, at the adapter's nominal 1 ms delivery cadence, which is the setting
of its latency timer. The FT232H documents latency timer values of 0 to
255 ms and the Linux driver accepts 0, so whether this adapter delivers
sooner at a lower setting is a question for measurement. This experiment
answers it torque-off without changing a production default. A lower setting
is an experimental possibility, not a promise of 1 kHz, and its results
qualify no rate.

**The setting on this host.** A udev rule sets the timer to 1 ms when the
adapter is plugged in. The driver's sysfs attribute shows its cached value,
or 1 whenever the port's low-latency flag is set, and with that flag the
driver writes 1 ms whatever value is stored. `tools/latency_timer.py read
<tty>` (as root) therefore reads three values: the sysfs value, the value
the adapter itself returns to a Get Latency Timer request, and the
low-latency flag. `set <tty> <ms>` writes the sysfs attribute and reads all
three back. Opening the port makes the driver send the adapter a reset, so
the adapter's value is read after the tool's own open, and again after each
run.

**The session.** `tools/trace_session.sh --latency-steps 1,0,1` makes each
listed run once per step, in order, each untraced and then traced. A 0 ms
result is thus bracketed by 1 ms runs of the same condition: an improvement
that follows the setting goes away again at 1 ms.
- Before anything is sent, the session reads the starting value, which it
  restores later, and refuses if the low-latency flag is set or the adapter
  and sysfs disagree.
- Before each run it sets the step's value if it changes, and checks that
  sysfs and the adapter both hold it with the flag clear; after the run it
  checks again. A run that fails either check is voided and stops the
  session.
- The probe expects the step's value (`x7_bus_timing
  --expect-latency-timer-ms`), so its own bus-profile check refuses any
  other. Without that option the bounded read's profile check requires
  1 ms, as before; the 100 Hz path reads no latency timer.
- So once the first stepped run begins, the session owes the starting
  value back, whether or not it had to set a step: a run opens the port and
  can change the adapter's value. On every exit (the end, a stop, Ctrl-C,
  SIGTERM, SIGHUP or any other exit of the script, all through its EXIT
  trap) it reads the adapter, verifies that the starting value is in effect
  and sets it again if not. A cleanup that leaves another value fails the
  session: an otherwise successful one exits 3, and a stop or an
  interruption keeps its own status. Only a SIGKILL or a crash skips the
  cleanup, leaving the step's value until the timer is set again (the udev
  rule sets 1 ms at the next plug-in). The run records carry every check
  and the cleanup's result.

**Measurements.** For each run, compared across the steps: the probe's
read and complete-cycle timing, criteria, errors and recoveries
(`tools/timing_summary.py`, as before), its CPU duty (in the summary), and
the xHCI interrupts around the probe. For each traced run,
`tools/trace_correlate.py` adds the adapter's delivery cadence, the replies
split across deliveries, the receive path and the host's USB work (the xHCI
interrupt handlers and the USB completion work in the run). The xHCI counts
include every device on the controller. For the 500 Hz feasibility analysis
(reviewer decision, 2026-11-08) it also gives each read's host
OUT-completion phase: where its instruction's OUT completion fell, on the
host, between the host's bulk IN completions around it, and per tenth of
that interval the reads, the share whose reply arrived in more than one
completion, and the median times to the first completion and of the host
transaction. It is a retrospective association, since the following IN
completion is part of it: it is not a predictor available before sending,
and it establishes neither a cause nor a remedy. A scheduling proposal
would have to estimate the phase from what is known before sending, and
validate that estimate independently. It also counts the cycles whose
host work exceeds T6a's 0.7 T, each with its largest measured part.

**Plan.** First, short quiet runs (18 runs, about 15 minutes):

```
tools/trace_session.sh --latency-steps 1,0,1 --duration 20 \
  --runs current:500:rt-idle:fast-motion,current:1000:rt-idle:fast-motion,cbp:1000:rt-idle:fast
```

Then, after those results are reviewed, repeated loaded comparisons, for
example `--latency-steps 1,0,1,0,1` under rt-loaded and rt-loaded-excl.
Both watchdogs, the receive budget and guard, and the qualification
thresholds stay unchanged; a lower setting in production would need its
own decision and fresh qualification evidence.

**Phase 1, 2026-11-06.** At `50e1169`, the plan above began with
current at 500 Hz. Both 1 ms runs completed: 10,000 cycles each, none
skipped, no transport error, T6a failed (host work p99.9 1.764 and
1.865 ms). At 0 ms, which the adapter held before and after the run, the
probe's startup inventory got no matching reply to its first ping (servo
2), so the probe exited before its timed run and the session stopped; the
other runs were not made. A traced repetition (attempt 2) recorded that
startup:
- the 10-byte ping was submitted over USB and its transfer completed,
  which shows the adapter accepted it, not that it went out on the RS-485
  line;
- all 1,754 bulk IN completions in the next 35 ms, about 49 per
  millisecond (the xHCI controller took about 16 interrupts per
  millisecond), carried only the adapter's two status bytes, with no
  receive-error flag, and the kernel trace's xHCI events agree;
- the 1 ms run's identical startup received the 14-byte reply 0.58 ms
  after its ping.

The failure lies before the adapter delivered any reply byte to the host,
so the TTY handoff cannot explain it. Whether the ping reached the line,
the servo replied, or the adapter received and held or dropped a reply is
not resolved, and no receive-only tap of the line is available (D7). The
failure came before the bounded read and the real-time worker started, so
it says nothing about cycle timing at 0 ms. Each session's cleanup restored
1 ms, and 1 ms stays the working setting.

**Hold diagnostic.** `sudo tools/latency_hold.py`, from the repo root,
asks whether the adapter at 0 ms received the reply and held it. On one
serial descriptor, open throughout, it:
1. checks that the adapter holds 1 ms (sysfs, the adapter's own value and
   the low-latency flag read through that descriptor), and pings servo 2
   as a baseline that must get a matching reply;
2. drains and flushes the input, sets 0 ms and reads it back, sends
   exactly one ping and observes the input for 50 ms;
3. sending nothing, and with no flush, reset, close or reconfiguration of
   the port, sets 1 ms, reads it back and observes another 50 ms;
4. sends a fresh control ping at 1 ms and observes 50 ms.

It sets the timer through sysfs and reads it back without opening the
port again, and only pings go to the servos. usbmon records the adapter's
traffic throughout, and every write and read is logged with its raw bytes
and `CLOCK_MONOTONIC` time (`rate-hold-<date>-<attempt>.csv`, with
`.csv.run.txt` and `.csv.usbmon.pcap`). A matching reply after the switch,
before anything further is sent, supports delayed delivery by the adapter,
though it does not timestamp the adapter's reception; no payload leaves
reception, transmission and dropping unresolved. A ping not written whole
aborts the sequence, as does any other transport error: no ping is
retried, so the experimental ping stays the only one. The result counts as
evidence only if tcpdump stopped cleanly, the capture holds packets and
tcpdump reports no drop; otherwise it is marked invalid and the
diagnostic exits nonzero. From the verified start on, the cleanup verifies
or restores 1 ms on every handled exit, before the recorder stops, with
the session's exit statuses (3 when the cleanup cannot leave 1 ms).

**Hold result, 2026-11-06.** At `d761371` (`rate-hold-20261106-1`):
- the baseline ping at 1 ms got the servo's reply 1.07 ms later;
- at 0 ms, held by the adapter (sysfs, its own value, the flag clear), the
  one experimental ping was written whole, and for the next 50 ms the
  adapter returned 2,495 bulk IN completions, about 50 per millisecond,
  each with only its two status bytes;
- with nothing sent and the port left as it was, the timer was set to
  1 ms again: the deliveries returned to one per millisecond, and the
  first, 0.77 ms after the setting request, carried the complete 14-byte
  reply from servo 2 (CRC good), which the diagnostic read 0.46 ms after
  the setting read back;
- the control ping at 1 ms was answered as usual. tcpdump stopped cleanly
  with no packet dropped, and the cleanup verified 1 ms.

This strongly supports delayed delivery: the servo replied, and the
adapter delivered the reply only once the timer was 1 ms again. No payload
was delivered for 50 ms after the ping; when the reply reached the adapter
is not measured. So 0 ms is not a faster setting here, and 1 ms is the
lowest working setting tested on this adapter and software configuration.
This closes the latency timer experiment and keeps 1 ms as the working
setting; it does not establish a general limit of the transport.

### Receive placement and late-wake diagnostics (torque-off, 1 ms)

These diagnostics follow the host-side tails that faster USB delivery would
leave: where the adapter's interrupt, its USB completion and the TTY work
run, and what the worker did in the T5 cluster's late sleeps. The latency
timer stays at 1 ms, and the scheduling settings and qualification criteria
are unchanged.

**Placement.** `tools/trace_correlate.py` follows each read's last USB
completion by identity: the xHCI interrupt's CPU, the USB completion work's
context (at interrupt exit or in ksoftirqd), the TTY work's kernel worker and
its CPU, and that worker's waking after the work was queued (or that it was
already awake). The queue wait splits into the time before the waking, the
time from the waking to the worker's first run after it (its wait in the
run queue), and then its time running, preempted (off its CPU, runnable)
and asleep until the work began, with summaries by CPU and for the waits of
100 us or more. A read whose work was already pending has no queue time and
is not split. The traced runs of 2026-11-06 already show:
- every xHCI interrupt on CPU 7, and the USB completion work at the
  interrupt's exit (in ksoftirqd for at most 4 reads per run);
- the TTY work on 15 to 26 different CPUs per run;
- in 95% or more of the reads, the kernel worker asleep when the work was
  queued and woken within about 0.2 us (p50), then waiting for its first
  run: 101 and 307 us at the median under the full load (1000 and 500 Hz),
  up to 3.8 ms;
- in the waits of 100 us or more, that wait for the first run the largest
  part (16,966 of 17,909 at 1000 Hz and 19,055 of 19,993 at 500 Hz under
  the full load), the worker's preemption after it had started running
  the largest in 563 and 678 of them; its first run followed stress-ng
  under the full load, and also desktop tasks (a browser renderer, the
  terminal, an editor) under the `-excl` load; idle, it followed the CPU's
  idle task, consistent with the CPU leaving an idle state (p99 62 to
  100 us).

**The profiled mode.** `tools/trace_session.sh --modes ...,profiled` (rt-*
conditions only) records what a traced run does, plus:
- the probe threads' timer arming (`hrtimer_start`, `hrtimer_cancel`), sleep
  calls (`clock_nanosleep` and `nanosleep` entry and exit) and page faults,
  each filtered to the probe's `comm`, and the workqueue's activations;
- perf's samples of the real-time CPU on the mono clock, with call chains
  unwound from a bounded copy of each sample's user stack, since the
  Release build keeps no frame pointers (`perf record -k mono -C <cpu>
  -F 999 --call-graph dwarf,4096`); the worker's samples are then written
  as text twice, their sampled instruction pointers and their call chains,
  and perf's sample, lost and throttle counts go to the run record.

`tools/trace_correlate.py` then associates each sleep (from the previous
cycle's end to the timer's waking) with exactly one sleep call of the worker
and, after it and before the expiry, an arming of the expiring timer (the
same hrtimer) whose expiry lies between the scheduled wake and the expiry; a
cancelled and re-armed timer counts its last arming. Missing, duplicate or
inconsistent evidence fails the run, as does a profiled run without these
events. It gives every cycle's time from its cycle-end stamp to the sleep
call, and for the latest wakes the call, the arming, the page faults and the
worker's samples in between: their sampled instruction pointers, and the
most common call chain, which is only as good as the unwinding. It also
counts the samples that fall in the worker's on-CPU time in the trace (the
clocks' alignment). The cycle-end stamp precedes the cycle's bookkeeping and
the sleep call, and not being descheduled does not show that the probe's
code ran: the samples show where the worker was. The sampling interrupts the real-time CPU, so the
untraced and traced runs of the same condition are the controls for its
interference. The traced mode's events are unchanged, so its runs compare
with the earlier ones.

**Plan.** First a rehearsal over `dxl_emu` of the profiled mode, since its
events, their `comm` filter and perf are new on this host (with the emulator
link and the adapter's sysfs directory as before). It must show the events
parsed and associated, the worker's samples parsed and inside its on-CPU
time, no lost or throttled samples, and call chains that reach the probe's
own functions (else the stack copy is too small or the unwinding fails):

```
tools/trace_session.sh --port <emulator link> --usb-device <adapter sysfs dir> \
  --modes untraced,traced,profiled --runs current:1000:rt-idle:fast-motion --duration 10
```

**Rehearsal result (2026-11-06).** The three runs completed. In the profiled
run:
- each of the 9,477 sleeps had its sleep call and an arming of its expiring
  timer, none missing, duplicate or inconsistent;
- perf stopped cleanly with 8,722 samples of the real-time CPU, none lost and
  none throttled (`perf report --stats --no-skip-empty`), and the worker's
  8,609 were written with and without call chains;
- 8,608 of the worker's samples fall in its on-CPU time in the trace; the
  other, 0.6 us after its switch-out event, is in the context switch itself.
  Shifted by 2 us, 39 to 62 fall outside (the worker was on its CPU 51% of
  the time): the check is sensitive to a 2 us displacement and the clocks
  were consistent as observed, which is not a guaranteed accuracy bound;
- 8,377 call chains (97%) reach `Probe::run` and the thread's start through
  the bounded read and the SDK's `readPort`, every kernel frame resolved. Of
  the rest, 94 taken at a system call's entry or exit have no user
  registers, and 138 stop at particular instructions (in libc's `read` and
  system-call code, `steady_clock::now` and the vDSO). None of those is
  deeper than a complete chain, and the worker's stack spans at most 632
  bytes, so the 4096-byte copy does not limit them.

The analysis fails its association check over the emulator, as the traced
run does, because a pty has no USB transfers. The first version of the
record kept only perf's nonzero counts, so no lost or throttled sample
showed as an explicit zero; the record now names each count. In this idle
run the time from a cycle's end stamp to its sleep call stays near 0.2 us,
so the latest wakes hold no samples. At 999 Hz, a 0.5 to 1.6 ms window of
the T5 cluster holds 0 to 2 samples, so the cluster's call context comes
from its cycles together.

Then, torque-off on the arm, each condition untraced, traced and profiled
(9 runs, about 14 minutes):

```
tools/trace_session.sh --modes untraced,traced,profiled \
  --runs current:1000:rt-loaded:fast-motion,current:1000:rt-loaded-excl:fast-motion,current:500:rt-loaded:fast-motion
```

The T5 cluster came from a traced 1000 Hz run under the full load, so the
loaded conditions are repeated; a profiled run may not reproduce it.

**Matrix result (2026-11-06).** The nine runs completed torque-off, with
complete logs and no read or write failure, and every traced and profiled
run passed the association checks with no recorder failure, lost trace
event or dropped usbmon packet; no rate is qualified by them. Two qualifications apply to
the data:
- the 500 Hz full-load untraced and traced runs had an uncontrolled
  background load (`exiftool` in their task snapshots), so they do not
  estimate the tracing's cost;
- the 1000 Hz full-load profile lost no sample, but the 500 Hz and the
  1000 Hz `-excl` profiles report lost records (72 and 3), so their
  sampling is degraded: their trace timing stands, and missing samples
  cannot show that some code did not run. The worker's samples outside its
  on-CPU time all fall within 5 us after one of its switch-outs.

During the 1000 Hz full-load profile the kernel logged, once, that its
frame-pointer unwinder met a bad frame pointer in gnome-shell's KMS thread
(code that includes the out-of-tree NVIDIA module), then lowered perf's
maximum sample rate to 31,000 Hz after one sampling interrupt took 6.3 us.
The warning's cause and its effect on timing are not established, and the
lowered maximum staying above 999 Hz does not show the profiling's
overhead to be negligible.

- **Receive path.** The earlier result repeats: in the 1000 Hz full-load
  profile, the TTY kernel worker's wait for its first run after its waking
  was the largest part of 15,982 of the 17,125 queue waits of 100 us or
  more (p50 88 us, p99 984 us), and its preemption after it had started
  the largest in 684.
- **Late sleeps.** The earlier pattern did not recur: in the three
  profiles, no sleep call came more than 10.4 us after its cycle-end stamp
  (101,659 sleeps). What the worker did in the earlier cluster stays open.
- **Late timer service.** The latest traced wakes came instead from the
  worker's timer being served late on CPU 3 after the worker had reached its
  sleep. In each of the 127 traced or profiled cycles whose timer was
  served 50 us or more late, CPU 3 recorded no timer interrupt between the
  due time and the service, and in 125 no event at all (two had a few
  scheduling events). With CPU 3 idle (75 cycles, 72 of them in the
  1000 Hz `-excl` profile) the delays were 50 to 198 us; with a task
  running there (52 cycles: stress-ng, `exiftool`, desktop and browser
  threads, gnome-shell's KMS thread) 51 to 941 us, the longest in the
  500 Hz full-load traced run. Late service does not establish its source
  or an interrupts-off duration: interrupt masking, NMIs, SMIs and the
  timer's programming all remain possible (the kernel's
  [timerlat documentation](https://docs.kernel.org/trace/timerlat-tracer.html)
  distinguishes these sources of IRQ latency).
- **Untraced T5 failures.** The 500 Hz full-load untraced run (42 wakes
  0.5 ms or more late, all in its first 7.4 s, max 1.765 ms) and the
  1000 Hz one (max 0.944 ms) failed T5. Without a trace they stay
  unattributed: an early cycle-end stamp does not show that the sleep call
  followed promptly.

**Next.** The timer service diagnostics below, with C2 left enabled.

### Timer service diagnostics (design, torque-off, 1 ms)

These diagnostics follow the late timer service of the profiled matrix: for
each of the worker's sleeps they lay out, on CPU 3, the sleep call, the
arming and cancellation of the timers that program its clock event device,
the deadline that device was programmed with, the CPU's idle-state
transitions, the timer interrupt's entry and the worker's waking and
switch-in. They are torque-off diagnostics: C2 stays enabled, the latency
timer stays at 1 ms, there is no DWARF sampling, and the scheduling
settings and qualification criteria are unchanged.

**Starting points** (checked 2026-11-07, without root):
- CPU 3's clock event device is `lapic-deadline`. The TSC is constant,
  nonstop and of known frequency (3187.2 MHz), and the APIC timer keeps
  running in idle states (`arat`), so no broadcast timer stands in for it.
- In the upstream 7.0 source, `lapic_next_deadline` writes the TSC deadline
  MSR (0x6e0) with `wrmsrq`, which on bare metal reaches the `write_msr`
  tracepoint; disarming the timer writes 0 there. This kernel's patches may
  differ, so the rehearsal must show the writes.
- The kernel has no irq-off tracing (no `irqsoff` tracer), so interrupt
  masking cannot be observed directly. It has kprobe events and the
  timerlat and osnoise tracers; the integrity lockdown permits tracefs and
  perf.

**Capture** (`tools/trace_session.sh --modes timer`, on rt-* conditions,
paired with an untraced control of the same condition):
- the traced mode's events, so that the existing association checks still
  apply;
- the worker's sleep calls by name (`syscalls:sys_enter_clock_nanosleep`,
  `sys_exit_clock_nanosleep` and the `nanosleep` pair, filtered to the
  probe's command name), as the profiled mode records them: the sleep
  checker's evidence, which the raw system calls below (numbers only) do
  not replace;
- the arming and cancellation of every timer (`timer:hrtimer_start` and
  `timer:hrtimer_cancel`, with the timer's address, function, expiry, soft
  expiry and mode) and each timer's clock (`timer:hrtimer_setup`, or
  `hrtimer_init` on older kernels), on all CPUs, since a timer started
  elsewhere can later expire on CPU 3;
- CPU 3's deadline writes (`msr:write_msr`, filtered to MSR 0x6e0), its
  idle-state entries and exits (`power:cpu_idle`, and `power:cpu_idle_miss`
  if present), NMIs (`nmi:nmi_handler`), and its system calls and page
  faults (`raw_syscalls:sys_enter` and `sys_exit`,
  `exceptions:page_fault_user` and `page_fault_kernel`), each restricted to
  CPU 3 by its event filter, so that the task on CPU 3 at a deadline is
  known to be in a system call or not;
- process creation, execution and exit on all CPUs, for the background load;
- a second trace instance (`tools/tsc_instance.sh`, through sudo) on the
  `x86-tsc` clock, restricted to CPU 3, recording its `local_timer_entry`,
  which the main trace records too: the anchors. It starts after the main
  trace and stops before it. Its start's status says whether it created the
  instance: one it did not create (it exists, including one another session
  made after the session's own check) is never removed; one it created is
  removed (verified) on every exit, even if a later step of the start
  failed; after a status that does not say (sudo's own failure, an
  interruption), an instance found is left in place and reported. Whether it
  exists is answered only from a directory listing the helper read in full
  (`present` or `absent`, status 0); a check that gives no answer (sudo's own
  failure, whose status 1 the helpers once gave for absence, or an unreadable
  tracefs) is never taken for absence: nothing is removed, the instance stays
  owed and is checked again at every cleanup and at exit, and the run record
  names it until its absence is confirmed. Before anything is sent, an
  instance left by an earlier session, or one whose absence is not
  confirmed, refuses the session;
- a timekeeping record (`tools/tk_recorder.sh`, through sudo; the certificate's
  evidence): kprobes on `ntp_adjtimex` and `ntp_clear` and on the entry and
  return of `update_wall_time`, in a private instance on the mono clock with
  its own buffer ((duration + 30) x 2000 x 64 bytes per CPU, at least 1 MiB),
  counted in the buffer read-back, the memory budget, the stage and its
  inventory. It is armed after the main trace is confirmed running and before
  the TSC instance: the return probe is enabled before the entry probe, every
  enable state is read back, then 0.5 s of settling updates and the first state
  read (`tools/tk_record.py state`, `adjtimex` with modes 0 and
  `ADJ_OFFSET_SS_READ`, which change nothing). After the TSC instance is read
  come the last state read, the second read-back, the entry probe's disable
  before the others, the hit, miss and buffer counts, `trace-cmd extract -B`
  and `trace-cmd report -t` (nanosecond stamps, which the parser requires), the
  kernel log's scan, and the removal, with ownership and checked existence as
  for the TSC instance; a recorder still owed stops further runs. Every use
  of trace-cmd runs under one launch (`tools/tk_record.py launch`): the dynamic
  loader's and libtraceevent's search inputs (every `LD_` variable, `HOME`,
  `TRACEEVENT_PLUGIN_DIR`, `TRACECMD_PLUGIN_DIR`) are removed, so that it loads
  only the system's libraries and the plugins of libtraceevent's compiled-in
  directory (with `HOME` unset, libtraceevent stops before the home directory
  and trace-cmd's source-tree directory; the main trace's record alone runs
  under `stdbuf`, whose preload only line-buffers its output). trace-cmd's
  identity (`tools/tk_record.py tool`: the file the session runs, resolved once
  at its start so that sudo runs the same file, its SHA-256, its package,
  version and architecture from dpkg, and its runtime under the launch: the
  shared objects the dynamic loader resolves for it, the trace libraries among
  them and every plugin libtraceevent would load, each by file, SHA-256 and
  package, the directories searched and any `/etc/ld.so.preload`) is read
  before the main trace starts and after the recorder's report; one not read
  before the run stops the session before anything starts, and one not read
  after it is a tracer failure. A
  failed step is a tracer failure and leaves that file absent or partial, never
  an empty record, and an interrupted run collects what it can before it
  removes the recorder. Its user-space helpers are pinned to the tracer CPUs;
  the kernel probes run on the CPU doing each update, CPU 3 included. The main
  trace's timer event is read as enabled before the TSC instance starts, and
  the run record keeps that read;
- bracketed TSC and CLOCK_MONOTONIC pairs (`build/apps/tsc_mono_pairs`:
  the tightest of 16 reads every 10 ms) on a tracer CPU throughout, and on
  CPU 3 for 0.5 s before and after the probe;
- CPU 3's SMI count in 10 ms intervals (`perf stat -e msr/smi/ -C 3 -I 10`),
  anchored by a monotonic time before it starts and the time its first
  interval is seen. An SMI in the interval of a late service is recorded as
  such; it does not locate the delay;
- the BOOTTIME and TAI offsets from CLOCK_MONOTONIC (with the REALTIME offset
  of the clock pair), to convert timers on those clocks;
- each buffer's losses (overrun, commit overrun and dropped events) in the
  run record, for the main trace from `trace-cmd report --stat` (every
  traced mode, over every online CPU) and for the instance from its CPU's
  stats. A failed read, a CPU missing from the stats, or a counter missing
  or not a number records the count as unknown and stops the session as a
  tracer failure: missing evidence is never recorded as zero losses. A
  positive count in either buffer invalidates the capture and stops the
  session the same way, never with a timing verdict (reviewer decision,
  2026-11-08);
- the fair server's probe events (Stage 1, approved 2026-11-08): kprobe
  events in the session's own group (`rtctrl_fs`), defined by
  `tools/fair_server.py` from offsets in the kernel's BTF and written by
  `tools/fair_server_probes.sh` through sudo. They record the deadline
  class's pick (`pick_task_dl`, with CPU 3's fair-server state at its entry
  and the task pointer it returns), the fair server's own pick
  (`fair_server_pick_task`, its state and the task pointer it returns),
  `set_next_task_fair` (a fair task set next through its class, its
  pointer never NULL, with its pid: the task's identity; the CFS fast path
  can bypass it, so a selection may stay unidentified), and
  `dl_server_start`, `dl_server_stop` and `dl_server_timer` with the
  server's state before each call. No probe reads through a pointer that
  can be NULL: the two return probes record the pointer alone (the first
  rehearsal's 56,712 kernel page faults of the worker came from such reads).
  The picks are filtered to CPU 3, the
  server's calls to CPU 3's server, wherever they run. The session defines
  them once, before anything is sent, and refuses a group left by an
  earlier session or one whose absence is not confirmed; it records the layout, the definitions and each event's
  format, and each run's hit and miss counts (`kprobe_profile`) right before
  and after the probe. Counts that cannot be read stop the session as a
  tracer failure. The group is removed (verified) on every exit, and a
  failed removal fails the session (status 3); a group whose definition had
  an unknown outcome is left in place and reported, with the commands that
  remove it by hand, and one whose existence is not confirmed stays owed as
  for the TSC instance. Whether each fetch argument reads as intended is
  confirmed in the rehearsal before any use on the arm.

A known unrelated batch workload (`tools/batch_workloads.sh`, by process
name: `exiftool`, the package tools, compilers and build tools, the desktop
indexers and LocalSearch's extractor) refuses the session at its start and
stops it before a run; the session never stops a process. A process is
exempt only when it is positively identified, by its NUL-separated
arguments, as unattended-upgrades' shutdown helper (`unattended-upgrade-shutdown
--wait-for-signal`, which only holds shutdown while an upgrade runs); an
upgrade itself, and a process of a listed name whose arguments cannot be
read while it exists, still refuse. The indexer refuses whatever its CPU
use, since a session's own output under `$HOME` can wake it. This is a
check for known workloads, not a guarantee against all background activity.

The operator quiets the resident services with `tools/quiet_host.sh stop`
and restores them with `tools/quiet_host.sh restore` (reviewer decision,
2026-11-07). It records each service's state first; it stops
`unattended-upgrades.service` only once no upgrade is running (its units and
processes checked, since the helper asks a running upgrade to stop), masks
`localsearch-3.service` for this boot unless it is masked already, stops it,
and checks that its extractor has exited. Restoring removes only the
runtime mask it added and starts only the services it stopped. Its record
(`$XDG_RUNTIME_DIR/rtctrl-quiet-host.state`, cleared at reboot) goes into
every run record as the session's temporary host changes. Its record is
written ahead (the states found, then each intended change before it is
made, every write checked); a state it cannot read refuses before any change
or fails a verification; an interrupted stop or restore keeps the record,
and restore, which checks each unit's actual state, can always follow and
be repeated. These safeguards were reviewed and the tool cleared for this
workflow (reviewer decision, 2026-11-08). The reviewed manual procedure
remains an alternative: confirm that no upgrade runs, stop the upgrade
service, mask for this boot and stop the indexer, and check that its
extractor has exited. Either way, the services are restored after every
session, also after a failed or interrupted one.

**Conversion and its bound** (`tools/timer_clock.py`, design revision 53,
frozen 2026-11-11; revision 53 of the analysis replaces interval-pattern
matching, and the reports name the revision). An anchor is identified by the
deadline written for its interrupt: each interrupt's slot holds the earliest
non-zero deadline written since the previous CPU 3 entry and after the last
write of zero, and each TSC stamp of the instance goes to the last slot at or
before it, one stamp to a slot. The deadlines are TSC counts, so this uses no
conversion. A zero write fails closed: a slot holding only a pending deadline
needs the sampler to prove it expired before the zero. The sampler is only an
independent check of the pairs, with sign checks at every pair. At most 0.1% of
the anchors may be left out, only the ones that set the bound's maximum, and U
is recomputed from the final set.

U bounds the error of every converted value, not a sample of them. For each
interval it is the larger of the two anchors' measured stamp differences plus
the sampler's bound, plus the chord over the interval and the clock-read
difference; U is the largest over all intervals. The slope range and the
10 ms clock-read maximum in it come from a certificate for that acquisition,
from the timekeeping record: the audited kernel (7.0.0-42-generic, its config
hash); the audited trace-cmd (3.3.3-1ubuntu2, amd64, the installed executable's
SHA-256) with its audited runtime, below, for every use the certificate rests
on, the acquisition, the recorder's extraction and report, and the main trace's
report that the analysis read, a later replay's included (a replay runs
trace-cmd under the same launch; a saved `--report` needs `--report-tool`, the
identity of the trace-cmd and runtime that wrote it, which `tools/tk_record.py
report` writes beside the report; an identity missing, unread, of another build
or with another runtime, or with a field the certificate does not know, is
UNSUPPORTED until a new review); clocksource `tsc` before and after with a
clean kernel log;
a complete record (every probe enabled before and after, no misses, no losses,
hits equal to records, each file read whole: a row that repeats, contradicts
another or is not understood makes it unreadable, never overwritten or
skipped); no modifying timekeeping call from arming to the last
state read, settled updates before the first read, no unmatched update entry
up to the end of the interval, every overlapping update within 10 ms, unchanged
frequency, constant and tick, the nominal 10,000 us tick, no `ntp_tick_adj` boot
argument under the kernel's own parsing, and the offset recurrence between the
two reads. The sampler brackets used and the conversion span must lie inside
the interval between the state reads. A hit with no record, as when a return
probe's instance is still outstanding at its disable (the kernel counts it and
writes nothing), makes the acquisition UNSUPPORTED rather than being explained
away.

The supported trace-cmd runtime (reviewer's decision, 2026-11-11) is pinned by
file and SHA-256 in `tools/timer_clock.py`; package versions are recorded
beside each file:

| Part | Package | Version | Files |
| --- | --- | --- | --- |
| trace-cmd | trace-cmd | 3.3.3-1ubuntu2 | `/usr/bin/trace-cmd` |
| libtraceevent.so.1 | libtraceevent1 | 1:1.8.7-1 | `/usr/lib/x86_64-linux-gnu/libtraceevent.so.1.8.7` |
| libtracefs.so.1 | libtracefs1 | 1.8.2-1ubuntu1 | `/usr/lib/x86_64-linux-gnu/libtracefs.so.1.8.2` |
| Plugins | libtraceevent1-plugin | 1:1.8.7-1 | the 13 plugins of `/usr/lib/x86_64-linux-gnu/traceevent/plugins`, the only directory searched |

The sanitized base launch must leave no search input, and the host must have no
`/etc/ld.so.preload`; the main trace's record child adds only `stdbuf`'s own
line-buffering preload after it, the one documented exception, which the
recorded inputs do not describe. The session checks the launch's producer by its
own exit status and requires its words to be exactly the prefix the session
builds for its environment (`env`, then `-u` for each fixed input and every `LD_`
variable set); a failed producer or a truncated, malformed or extended prefix
refuses the session, traced or not, before anything is created. The
generic libraries the loader also resolves (libz, libzstd, libc) are recorded,
not pinned. Any other file, version, plugin or search input makes the
certificate UNSUPPORTED: changing the baseline (an upgrade of any of these
packages, an added plugin) needs a review of the changed source, as the
original audit did, and a revised baseline before its acquisitions can be
certified. The libraries' and plugins' source packages match the archive's
signed index for these versions.

Without a certificate the run fails. `trace_correlate.py --exploratory-slope-ppm`
computes a U labelled exploratory for archived captures, which stay failed and
keep their original verdicts. A deadline is converted by linear interpolation
between the retained anchors around it, and never extrapolated (outside them it
is INDETERMINATE). A U above 5 us fails the run; it is an engineering acceptance
target, not an accuracy claim, and the classification's boundaries include it.
The residuals of held-out anchors and of the sampler are printed as diagnostics
only.

**The deadline timeline** (`tools/timer_service.py`, through
`tools/trace_correlate.py` for a run recorded in the timer mode). The
observed MSR timeline is primary, and it follows the device's state, not
only its last write: a write arms the device for its deadline, a write of 0
disarms it, and a failed write leaves its state unknown. A TSC deadline
disarms itself when it raises its interrupt (Intel SDM, volume 3, the local
APIC timer's TSC-deadline mode), so the first local timer interrupt at or
after the deadline (less U) serves it, and the device stays unarmed until
the next write. An interrupt before the armed deadline leaves it armed;
before the first write the state is unknown. Firing and service are
separate observations: a deadline may fire while interrupts are masked, its
interrupt pending. So a write that replaces an armed deadline already due
(within U), or an unknown state, with no interrupt seen since, carries that
earlier generation as possibly pending, and the next local timer interrupt
may be its. A write's time is its tracepoint's, which follows the WRMSR
instruction; the record states it as traced.

**Residency and attribution.** Each timer arming is followed to its end:
the timer's next cancellation, expiry or arming. Its residency (the CPU
whose device it is queued on) comes only from the trace's evidence: the
CPU of its expiry, or the arming CPU of a pinned arming. The CPU that
recorded an unpinned arming is not its residency. A matching deadline write
on CPU 3 just after an arming there is noted as a hint, and the residency
stays unknown: the trace does not show that no device interrupt, NMI or
other context ran between them (the reviewer's decision of 2026-11-07; a
stronger rule would need a validated execution-context contract). Each deadline write is attributed to the one pending
timer resident on CPU 3 whose expiry, converted from its own clock, lies
within U of it. A timer resident elsewhere cannot be it; a matching timer
of unknown residency, a pending one whose clock cannot be converted, no
match and several leave the write INDETERMINATE. Timers armed before the
trace began are not seen.

**Classes.** For each paired sleep: the worker's sleep call, the arming of
its timer (its expiry E), the deadline whose interrupt served it (the one
armed just before the last local timer interrupt before the expiry, due at
the later of its value and its write), the device's state at E, CPU 3's
idle state and task. A sleep whose timer is served 50 us or more late is
described by its recorded evidence, not by a proven cause:
1. **late sleep call**: the worker entered its sleep call after E;
2. **timer armed late**: the call entered before E, but its timer was armed
   after it (the delay is inside the kernel's timer setup); without a
   recorded call entry, only the late arming is stated;
3. **deadline late**: the serving deadline was due more than U after E. The
   detail gives the device's state at E: armed for a later deadline,
   disarmed, or not armed because its last deadline had been served;
4. **interrupt late from idle**: the serving deadline was on time and CPU 3
   was in an idle state at it (the state, and since when);
5. **interrupt late while busy**: the serving deadline was on time and a
   task ran on CPU 3 (in a system call, or not, or with no system call
   recorded, or a kernel thread), with any NMI, device interrupt, other
   timer interrupt or SMI interval overlapping the delay. A task outside a
   system call may still have been in a page fault or an interrupt; one in
   the kernel may have masked interrupts, which stays unproven;
6. **INDETERMINATE**: the deadline cannot be converted (outside the anchors,
   or U not validated), the device's state at E is unknown, the serving
   interrupt follows no armed deadline, it came before the armed deadline,
   or an earlier deadline's interrupt may have been pending (the trace
   cannot tell which generation raised it).

A late service is a diagnostic result, not a failure of the run. The run
fails instead when the evidence the mode records is missing: CPU 3's
deadline writes, local timer interrupts and system calls, the worker's
sleep calls and timer armings, a valid loss count of zero for each buffer,
the TSC instance, sampler pairs on a tracer CPU and on CPU 3 before and
after the run (inside the anchors), a valid SMI count series covering the
run (a count perf could not take, or a gap, fails it), and the exerciser's
file when the record says it ran.

**Waits after waking and the fair server** (`tools/fair_server.py`, through
`tools/trace_correlate.py`; Stage 1, approved 2026-11-08). Every wake of the
sleeping worker on CPU 3, timer and read wakes alike, is followed from its
`sched_waking` to its switch-in; the completed `sched_wakeup` is reported
too. CPU 3's time inside the wait is split into overlapping execution
intervals at its switches, interrupts and softirqs, and each interval is
labelled by what occupied the CPU:
- **server-backed**: a normal task in validated server-backed dispatch. A
  selection is the fair server's own pick returning a task pointer that the
  deadline class's pick then returns; the pointer's pid comes from
  `set_next_task_fair`. It is dispatched when the next switch on CPU 3 runs
  that pid, or when the task was already running and stays on (a
  re-selection). Its interval may begin before the wake. The server's state
  read at the selection must show it active and not throttled;
- **unidentified**: a selection whose task's pid is not known (it was never
  set next on CPU 3): perhaps server-backed, never preemption delay;
- **contradictory**: a dispatch whose server state does not;
- **preemption delay**: a normal task already running at the wake and left
  running with no selection, only when the probes are in the capture and
  missed nothing;
- **normal unvalidated**: any other normal-task time;
- **higher class**: a real-time or stopper task, or a deadline-class
  selection the fair server did not make;
- **interrupt**, and **idle/switching**.

A wait of 50 us or more is classed **server-backed** only when server-backed
intervals cover it apart from interrupt and switching time (a 2 us
tolerance per label); **mixed** when they cover part of it beside another
mechanism; **other mechanism** when they cover none; and **INDETERMINATE**
when normal unvalidated, unidentified or contradictory time covers more
than the tolerance. The class says what occupied CPU 3, never a sole cause. A pick
returning no task, a selection followed by another switch, and a
contradictory state are counted apart, with the server's start, stop and
timer calls and CPU 3's deadline-class timers as context. Normal tasks
running without a recorded `dl_server_start` never invalidate a capture;
probes defined for a run that recorded no pick on CPU 3 do (their evidence
is missing), and so does any kernel page fault at a NULL-page address on
CPU 3 during the run, which the analysis counts in every capture. The
event does not establish its source (a probe's read through a NULL pointer
causes one, and so does another task's system call given a NULL pointer, as
in H2 attempt 6 below), so the trace has to show which. A capture without the probes, such as the earlier ones, gets
the same waits with its normal-task time unattributed.

**Rehearsal** over `dxl_emu`, before any run on the arm:
- **availability**: the session refuses before anything is sent if a
  required event or the `x86-tsc` clock is missing, or if a previous
  session's instance remains, and it lists the events' formats, CPU 3's idle
  states and its clock event device in the run record;
- **an exerciser** (`tools/timer_exerciser.py`, `--exerciser`, refused
  without `--port` and on a serial device): a normal-priority process pinned
  to CPU 3 that arms CLOCK_MONOTONIC timerfd timers at absolute expiries,
  cancels some before they expire, re-arms some earlier or later, and lets
  others expire, logging each action with its monotonic times and expiry.
  Each action is followed through its timer's armings: an arm is one
  arming for the requested expiry, no earlier arming pending; a re-arm is
  one, ending the round's pending arming by one cancellation in its call
  (none if that arming had already expired); a cancellation ends the
  pending arming of its expiry by one cancellation in its call, or finds it
  already expired (an earlier round's expiry does not count). An action
  need not write the deadline: while the arming is pending (at the call's
  end, or at its expiry if that came inside the call), the device must be
  armed (or, at the expiry, served) for a deadline no later than its expiry
  plus U, and a missing write is legitimate when an earlier deadline
  already covers it. An arming made after its own expiry (its arming at or
  after the expiry, less U; the exerciser's call came late) is counted
  apart, as armed after its expiry, when a successful deadline write after
  the arming and before the evaluation point has a deadline already due at
  that write (within U): the kernel's programming of an expired timer. A
  stale write or a future deadline leaves it unexplained (reviewer
  decision, 2026-11-07). After a cancellation the device may stay armed (a
  spurious interrupt follows) or be reprogrammed, but not disarmed. An
  arming whose residency is unknown (it neither expired nor was pinned) or
  another CPU is INDETERMINATE once its events have matched, and the report
  counts the INDETERMINATE results by reason;
- **requirements**, for the review: zero losses in both buffers; the anchors
  matched one-to-one with none missing or duplicated, any suspect anchors
  listed; the conversion covering the whole run; U at most 5 us; every
  exerciser action explained by the observed timer state, legitimate
  no-write cases included (the INDETERMINATE ones counted); the mode's
  evidence complete; and the untraced control as before.

The rehearsal's results are reviewed before targeted torque-off runs: each
an untraced control and a `timer` run, at 500 Hz under the full load (busy
CPU 3) and at 1000 Hz under the `-excl` load (idle CPU 3). A run with an
uncontrolled background load is kept, marked and excluded from controlled
comparisons. A C2 on and off comparison would be considered, and reviewed
apart, only after idle-state transitions have been observed.

**Rehearsal command.** With the emulator running (`./build/apps/dxl_emu
--link /tmp/rtctrl-rehearsal-tty --firmware 47`), Return Delay Time 0
written to its servos after each start (it starts at the factory 250), the
TSC sampler built (`cmake --build build --target tsc_mono_pairs`), and the
real-time priority limit raised in the starting shell:

```
for id in 2 3 4 5 6 7 8 9; do
  ./build/apps/dxl_inspect --port /tmp/rtctrl-rehearsal-tty write $id return_delay_time 0
done
tools/quiet_host.sh stop
tools/trace_session.sh --port /tmp/rtctrl-rehearsal-tty --usb-device /sys/bus/usb/devices/1-9 \
  --modes untraced,timer --exerciser --runs current:1000:rt-idle:fast-motion --duration 10 --outdir <dir>
tools/quiet_host.sh restore   # also after a failed or interrupted session
uv run --project tools tools/trace_correlate.py <dir>/<log>.csv
```

**Rehearsal result (2026-11-07, accepted).** At `bc8413b` (current,
1000 Hz, rt-idle, 10 s, with the exerciser; archived in `timer_rehearsal/`),
analysed with the conversion and exerciser rules above (`8a68857`). Both
buffers lost nothing. The kernel's deadline writes trace (38,720 on CPU 3),
the `CPU ==` filters hold, the instance's stamps are raw TSC counts, and the
SMI series covers the run with a count of 0. Of 29,117 anchors in each
buffer, 29,113 are paired and 4 are left out as suspect, their two stamps
disagreeing by 3.1, 3.3, 3.3 and 11.4 us; in each, the interval from the
interrupt's entry to its handler's first event was 3.3 to 11.5 us, against
typically under 1 us: a delay between recorded events whose cause the trace does not
establish. U is 2,580 ns over the whole run (held-out anchors up to
1,389 ns; the sampler's median residual -58 ns on a tracer CPU and
-1,680 ns on CPU 3, whose samples follow wake-ups from idle). Of the
exerciser's 6,180 actions, 834 are OK, 2,092 need no write, 235 were armed
after their own expiry (its call delayed by the real-time worker), 3,019 are
INDETERMINATE for unknown residency (739 with the write hint), and none is
unexplained. No worker timer service reached the 50 us classification
threshold (the probe's largest wake lateness was 98.869 us). The association
check's USB part fails over the pty, as expected, and validates nothing of
the USB path; the hardware USB checks stay strict. The two resident services
were stopped by hand for the session and restored. The targeted torque-off
captures follow.

**Targeted torque-off runs (approved 2026-11-08).** The reviewer approved
the four runs planned above: an untraced control and a `timer` run at
500 Hz under the full load and at 1000 Hz under the `-excl` load. The
exerciser is not used on the arm. The settings stay as they were: the
adapter's latency timer at 1 ms (no `--latency-steps`), C2 enabled, no DWARF
unwinding (no profiled run), and the criteria unchanged. The approval
covers this diagnostic workflow, not a higher powered control rate. Before
the session, every servo is torque-off, the arm is at rest and supported,
and the power cutoff is within reach; the TSC sampler is built and the
real-time priority limit raised in the starting shell (the session checks
both). Each run lasts the default 60 s, as in the matrix:

```
tools/quiet_host.sh stop
tools/trace_session.sh --modes untraced,timer \
  --runs current:500:rt-loaded:fast-motion,current:1000:rt-loaded-excl:fast-motion
tools/quiet_host.sh restore   # also after a failed or interrupted session
uv run --project tools tools/trace_correlate.py <log>.csv   # each timer run
```

**Repeat of the 1000 Hz `-excl` pair (reviewer decision, 2026-11-08).** Of
the four runs (archived in `timer_torque_off/`), the 500 Hz timer capture is
valid diagnostic evidence; the 1000 Hz `-excl` timer capture lost 48,404
events from CPU 3's main trace and is rejected as incomplete. Neither rate
qualifies. That pair is repeated first, at 20 s each and with an unused
attempt number, after the session's verdict for a positive loss count was
fixed. The event set and every other setting stay as they were, with C2
enabled, and every existing evidence check must pass. If events are lost
again, a narrower system-call tracing is designed and rehearsed before any
use on the arm:

```
tools/quiet_host.sh stop
tools/trace_session.sh --modes untraced,timer --runs current:1000:rt-loaded-excl:fast-motion \
  --duration 20 --attempt 2
tools/quiet_host.sh restore   # also after a failed or interrupted session
uv run --project tools tools/trace_correlate.py <log>.csv   # the timer run
```

**The repeat's results (accepted 2026-11-08).** The repeat (attempt 2, at
`8493d14`) gave a complete timer capture. No event was lost in either
buffer, all 39,010 of the TSC instance's anchors matched with none left
out, U was 1,270 ns, and the USB association check passed. The untraced
control reached 663.5 Hz and the timer run 559.7 Hz (measured). Both failed
T1, T2 and T6a, and the timer run also failed T5 (wake lateness p99.9
101 us, maximum 573 us). 1 kHz remains unqualified in the tested
configuration. The capture shows two sources of late worker wakes on
CPU 3, neither of them an established cause:

- 117 of the 11,191 sleeps were served 75.5 to 176.0 us late, every one
  from idle state 2 (`C2_ACPI`, declared exit latency 127 us);
- in two wakes, normal-priority tasks ran on CPU 3 before the woken
  worker: for 568.6 us at cycle 10068 (the T5 maximum; threads of the
  observer applications) and for 330.1 us at a read's wake. This is
  consistent with the fair server. Deadline-class timer events on CPU 3
  around both are an indication, not validated server state.

The capture's files are in `timer_torque_off/` of the
[archive manifest](../records/data-archive.md); the originals were removed
after the ledger check.

**Next steps (reviewer decisions, 2026-11-08).**

- 500 Hz feasibility has priority, without a promise of qualification.
- The fair-server instrumentation is approved as software, with four
  requirements:
    - it observes server-backed dispatch (validated server state and the
      selected task, correlated with the actual switch), not function
      entry;
    - it attributes each wait by overlapping execution intervals, allowing
      mixed and INDETERMINATE results, and never invalidates a capture
      because normal tasks ran without a recorded server start;
    - positive and negative controls on a rehearsal CPU validate the
      probes. The FIFO exerciser there has a self-imposed deadline and a
      supervisor on another CPU, and normal-task progress and observed
      server-backed dispatch are the evidence, not a fixed share;
    - the implementation and the rehearsal are reviewed before hardware.
      Attempts 3 to 5 at the repeat's condition then follow, as a labelled
      mechanism-reproduction experiment with the observer applications
      open.
- No cpuset partition yet. Observer placement outside CPUs 2 and 3,
  helper threads included, is compared first, without waiting for the
  fair-server result. An isolated cpuset also moves unbound-workqueue work,
  so its design would have to address the TTY receive path.
- C2 stays enabled in that baseline; a reversible C2 comparison is to be
  designed.

**Observer placement (approved for implementation 2026-11-08).** The
observers are the applications left open while runs are watched (their
process trees: the Claude Code processes, the browser, Dropbox and the
other desktop applications) and the probe's helper threads (its main thread
and log writer), not its worker. The session's tracers already run outside
CPUs 2 and 3, and kernel receive workers are outside this mechanism.
`tools/trace_session.sh --placements open,confined --observers <PID,...>`
makes each listed run once per placement, its log name labelled `-obs-open`
or `-obs-confined`. For a confined run, `tools/observer_affinity.py`:
- takes the operator's explicit PIDs and their descendants (the session's
  own tree is exempt), and before any change writes
  `<log>.csv.observers.jsonl`: every process's and thread's IDs, start
  times (a guard against reused IDs) and mask, flushed to disk;
- sets each thread's mask to its own less the excluded CPUs, writing the
  intended mask to the record (flushed) before each change and reading the
  mask back after it. A thread that holds only excluded CPUs, a process or
  thread whose identity (its `stat`), task directory or mask cannot be read,
  or a live thread whose change or read-back fails, refuses the run. Only a
  missing `/proc` entry (`ENOENT`, `ESRCH`) says a task exited; a
  permission, I/O or other read failure is never counted as an exit
  (reviewer, 2026-11-09);
- repeats that pass (approved 2026-11-09). A new thread inherits its
  creator's mask, so one created between a listing and its creator's change
  keeps the excluded CPUs (rehearsal 8, below). Each pass lists the trees
  afresh (the roots and every process found before, by ID and start time,
  even if reparented since, with their descendants, less the exempt trees,
  also listed afresh) and every live thread in them; a process or thread not
  seen before is recorded before any change to it. Confine ends after a pass
  that finds nothing new and every live thread off the excluded CPUs: every
  thread alive then held such a mask, so a thread one of them creates later
  inherits one too. Ten passes that do not settle, or a thread that holds
  the excluded CPUs again after confine confined it or found it off them (an
  application setting its own mask), refuse the run; the observers are
  restored before any probe starts. A placement is no freeze of the
  applications, so the checks below stay mandatory. Nor are repeated scans
  of `/proc` an atomic view of the trees: a process forked by a thread not
  yet confined (between a listing and its creator's change) whose parent
  exits before the next pass is reparented out of the trees before any pass
  sees it, so neither the passes nor the snapshot's parents attribute it to
  an observer (see the limitation below);
- records each thread's mask with its origin: the first pass's listing (the
  original mask), a later pass's thread still on the excluded CPUs (its mask
  before the change), or a later pass's thread already off them, which
  inherited a confined creator's mask and is left as found (its mask is no
  evidence of its original placement);
- starts the probe with its mask outside them, so only its worker pins
  itself to CPU 3;
- restores right after the probe, and on every exit. Restoration reconciles
  every recorded intent, completed or not (an interruption can fall between
  a change and its completion record): a thread that still exists with its
  start time and holds the intended mask goes back to its recorded mask,
  read back; one already at its recorded mask needs nothing. An exited
  thread, a reused ID and a mask the application changed during the run are
  left as found and reported. Any other error on a live thread, an
  identity or task directory that cannot be read, or a read-back that does
  not match, is unresolved: the record stays for a
  retry, the tool exits 1, and the session reports a cleanup failure (the
  run stops; an otherwise successful session exits 3) with the command to
  retry (reviewer findings, 2026-11-08). A thread found already off the
  excluded CPUs, and a thread or process created after the last pass,
  inherited its creator's mask, and a multithreaded process has no single
  original mask: it gets its process's original mask (for a process none of
  whose threads was found before its change, its recorded parent's; for a
  new process, its nearest recorded ancestor's) only when all that
  process's threads listed before their change shared one, and only while it
  holds that mask's confined form. Otherwise it is left at its mask and
  reported as left, never given an invented wider mask.

The run's halfway task snapshot lists every task's process, parent, start
times and allowed CPUs, closed by a completeness line; a live task it cannot
read makes the listing incomplete (its exit status 1), and an incomplete or
unfinished listing is no evidence. The snapshot's status and the check both
count: an incomplete listing, or a task of a recorded process (matched by ID
and start time) or of a descendant (by the snapshot's parents, the exempt
trees left out), or a probe helper, allowed on an excluded CPU, stops the
session as a placement failure. A snapshot is one moment, not the whole
run. A confined run is
untraced or in the timer mode. In a timer run, the session lists the same
lineage right after the trace started (`<log>.csv.lineage.txt`), and
`tools/trace_correlate.py` counts the time spent on the excluded CPUs by
every observer task (the record's, the lineage's, so that a task created
before the trace and absent from the record is one, and every task they
forked during the trace) and by the probe helpers. Any such time, a missing
or incomplete lineage (the session stops before the probe when it cannot list
one) or missing process events fails the analysis. The open runs leave the
observers as they are, and C2 stays enabled in both placements.

**Limitation of the placement evidence.** The passes, the snapshot and the
trace attribute a task to an observer only through what they saw: the
recorded processes, the parents in a listing, and the fork events of a
trace. A descendant that left the trees unseen (forked between two passes
by a thread not yet confined, its parent exiting before the next pass)
inherited the unconfined mask and is attributed by none of them, unless a
timer run's trace recorded its fork. A passing confined run therefore shows
that every observer task these mechanisms could attribute stayed off CPUs 2
and 3, not that every possible descendant was isolated throughout the run.
That is enough for the emulator rehearsal (reviewer, 2026-11-09); a stronger
isolation claim needs a separately reviewed approach.
The same-session open and confined comparison, with its run count and
labels, comes to the reviewer before it runs.

**Stage 1 rehearsal: the probes' controls.** Before the probes' evidence is
used on the arm, a rehearsal over `dxl_emu` validates them with positive and
negative controls on a spare CPU (`--fs-controls <cpu>`, refused without
`--port`, on a serial device, and on an excluded CPU). Before the timer
runs, `tools/fair_server_control.py` runs, on that CPU:
1. **positive**: a normal task that samples its own progress, and beside it
   a SCHED\_FIFO busy loop (priority 10) that stops itself at a deadline on
   `CLOCK_MONOTONIC` (3 s). Its supervisor, pinned to another CPU, kills the
   loop at the deadline plus 0.5 s if it still runs, and says so;
2. a settle of 2 s ending in a state check, then **negative-normal**: the
   normal task alone (3 s);
3. a settle of 2 s ending in a state check, then **negative-idle**: the idle
   CPU (3 s).

The phases are traced with the same probe definitions, filtered to that CPU,
with the tracers kept off it. The record holds the probes' hit and miss
counts and both buffers' losses (the loss rule applies), and
`tools/fair_server.py controls` judges the controls:
- **positive** passes when the trace shows the loop at a real-time priority
  on that CPU, and the normal task progressing and dispatched by the fair
  server while the loop ran. No fixed share (such as 50 ms per second) is
  required;
- a **negative** control's preconditions must be established first, from
  the server's state as read after the last call that could change it: no
  probe misses; no fair-server selection between the previous test and the
  control; and a state read at a deadline-class pick in that gap after the
  last `dl_server_start`, `dl_server_stop` or `dl_server_timer` entry there
  (an entry state precedes a call that may change it: the timer can set
  deferred-running and enqueue the server), the latest such state showing
  the server not in its running mode (a deferred server not
  deferred-running, or an inactive one). The tool ends each settle with a
  state check, `true` run under SCHED\_FIFO on that CPU, whose exit makes
  such a pick read the state. Without them it is INCONCLUSIVE,
  since the server's state can carry over from the positive test (active,
  throttled and deferred states differ). With them it passes when the
  server dispatched nothing while the normal task ran alone (which must have
  run), or selected nothing on the idle CPU, and fails otherwise. (A first
  rule required a recorded stop of the server; this kernel never called
  `dl_server_stop` in the rehearsal of 2026-11-08, its server going back to
  deferred, armed and throttled, or inactive, so the rule could never be
  met. A second accepted any recorded state, entry states included, which
  can precede an activation; reviewer findings, 2026-11-08.)

A failed control stops the session before the timer runs; an inconclusive
one is reported. The supervisor drains each child's output while it waits
for it, so a full pipe cannot block either side; every wait is bounded (a
normal task that does not say it is ready within 2 s ends the run), and a
signal to the supervisor kills and reaps every running child and marks the
output incomplete. Controls are never reused: if any file of the
attempt's controls exists (passed, failed, partial or interrupted), the
session refuses before anything is sent, and a fresh `--attempt` runs them
again (reviewer findings, 2026-11-08). The controls, the rehearsal's timer runs with the probes,
and the implementation come to the reviewer before any use on the arm.
With the emulator prepared as for the rehearsal above:

```
tools/quiet_host.sh stop
tools/trace_session.sh --port /tmp/rtctrl-rehearsal-tty --usb-device /sys/bus/usb/devices/1-9 \
  --fs-controls 5 --modes untraced,timer --runs current:1000:rt-idle:fast-motion --duration 10 --outdir <dir>
tools/quiet_host.sh restore   # also after a failed or interrupted session
uv run --project tools tools/trace_correlate.py <dir>/<log>.csv
```

**Stage 1 rehearsals (2026-11-08; not accepted).** At `3a2d12d`, attempt 3
(the controls, an untraced and a timer run) and attempt 4 (open and confined
placements, each an untraced and a timer run) ran over the emulator,
archived in `stage1_rehearsal/`. The three controls passed with their
preconditions established (all four of the normal task's server-backed
dispatches identified; confirming state reads 3.8 and 4.0 ms before the
negatives), and both confined runs confined and restored every observer
thread (about 1,240 each) with no failure. No timer capture passed every
evidence check: attempt 3's timer run lost 108,445 events on CPU 3, the
confined one 2,674,161, and all three failed the SMI coverage rule (largest
intervals 181.8, 48.3 and 1,008.0 ms). Each loss and each long SMI interval
coincided with trace-cmd and perf in uninterruptible sleep together, both
waking within 0.1 ms of each other (146 and 172 ms; 998 ms; twice about
43 ms, 5.000 s apart, in the open run, where the TSC sampler stalled with
them). Each long SMI interval equals perf's stall plus one 10 ms interval,
and each hole fits a blocked reader of the 16 MiB overwrite buffer. Those
are consistent with backpressure on the recorders; they do not identify the
stalled resource (writes to the ext4 output directory, memory reclaim or a
lock), and the cause stays provisional (reviewer, 2026-11-09). Split at the
recorders' `sched_wakeup` events, every one of those spans was blocked until
its wakeup in all but at most 0.02 ms, so none was mostly a runnable wait.
The earlier rehearsal (2026-11-07), written to tmpfs, lost nothing.

**Recorder staging (approved with three amendments, 2026-11-09).** Every run
of `tools/trace_session.sh`, untraced runs and the controls included, writes
its capture streams to a stage of its own (`tools/trace_stage.py`): a
private tmpfs mounted for the run under `/run/rtctrl-stage/` with `size=S`,
`noswap`, mode 0700 and the operator's ownership, on a mount point that is
the operator's with mode 0700 too. The streams are the
probe's log and summary, the trace with trace-cmd's per-CPU temporary files
(`--temp`), usbmon, the SMI counts, the TSC instance and sampler pairs,
perf, the exerciser's log and the snapshots. The run record and the
observer record stay in the output directory, so every recovery command
names a file that exists. The events, clocks, buffers, compression, recorder
CPUs and priorities, C2 and every evidence rule are unchanged; only where the
streams are written changes. The arrangement is diagnostic, not a
qualification of the production logging to disk.

- **Budget, per run and by mode** (provisional, empirical constants for the
  measured timer configuration): S = 1.25 (R + R/4 + A) + Q, with R =
  250 MB/s over the run plus 10 s of raw trace, R/4 its assembled output
  (4:1 compression is an assumption; measured 19:1 to 21:1), A the other
  streams the mode enables (at least 512 MiB in a traced run), and Q =
  64 MiB the guard's reserve. Untraced runs have no trace allowance (a 10 s
  untraced run at 1000 Hz gets 109 MiB); the controls are budgeted over their own
  duration (three phases, two settles and 10 s); perf's frequency and stack
  size are budgeted. Before the mount, `MemAvailable` must be at least
  S + K + H: K the trace arrays' data-plus-reader allocation, for each
  array and possible CPU (n + 1) subbuffers, with n = max(2, ceil(B / p)),
  B the requested bytes and p the payload of a subbuffer in the array's page
  header (4,080 bytes of a 4 KiB subbuffer on this kernel), and one more for
  the reader: 16,456 KiB per CPU for 16 MiB requested, 1.08 GB for a timer
  run's two arrays of 32 CPUs, bookkeeping and transient allocations
  excluded. H = max(8 GiB, 15% of `MemTotal`). A 60 s timer run needs about
  39.1 GB.
  The measured raw traces were 1.51 GB for a 10 s rehearsal run and 9.56 GB
  for hardware attempt 1's 60 s loaded run.
- **Before anything runs:** the mount is verified (a tmpfs exactly at that
  point, with `noswap` and the requested size, private) and marked as the
  session's. A refusal or failure at any step before the marker undoes the
  stage only while it holds no data, and the run does not start.
- **Before the probe:** trace-cmd's temporary files must be on the stage
  only, and every trace array must read back exactly as the kernel rounds
  the request, a check kept apart from K. Its geometry must be the main
  array's, read before any run (a missing or malformed page header refuses
  the session). Each configured CPU's `buffer_size_kb` must read
  floor(n p / 1024) KiB (16,387 for 16 MiB), and `buffer_total_size_kb` their
  sum, truncated per CPU before summing (524,384 KiB per array); otherwise
  the run stops before the probe. A guard on the
  tracer CPUs samples the stage's free space and `MemAvailable` every 0.2 s
  through the run and the assembly; below R/4 + Q free (what the run still
  writes after a stop, plus the reserve) or H/2 available, it stops
  trace-cmd (and perf) and records why. An untraced run has no recorder to
  stop: a trip there is recorded and fails the run. Because the stage holds
  Q on top of the budgeted streams, in every mode it stays above the bound
  when empty, with the budgeted streams while recording, and at the
  assembly's end. It is a guard, not a reservation. The record gives its
  sampled peak and a conservative bound (the usage just before trace-cmd's
  stop plus the assembled trace).
- **After every writer stops:** an immutable inventory (SHA-256 of every
  staged file) is written durably to the output directory
  (`<log>.csv.staging.inventory.json`) before any copy. Each file is copied
  to a partial name, fsynced, hashed as read back, compared and linked
  without overwriting, and its directory fsynced; the ledger
  (`<log>.csv.staging.sha256`, checkable with `sha256sum -c`) comes last.
  Nothing syncs the whole filesystem: Dirty and Writeback are recorded, with
  a bounded settling check. The stage is released (emptied, unmounted,
  removed) only when the ledger and every copy verify. Past the unmount
  nothing inside the mount point is read or deleted: the bare mount point is
  removed with rmdir, which removes only an empty directory, so a retry
  after a failure or an interruption there needs neither the marker (gone
  with the tmpfs) nor access to the directory. The capture is then
  judged from the published copies, failed captures being published the
  same way.
- **Failures:** a failed publication keeps the stage and names it with the
  retry; the capture is judged from the stage, so its own verdict is
  recorded apart; the session fails with status 3 (130 kept for an
  interruption). A failed release is a cleanup failure (status 3). A stage
  left by an earlier session refuses every later session, which never
  touches it. The retry, as the operator:

```
python3 tools/trace_stage.py publish --stage <stage> --outdir <dir>
python3 tools/trace_stage.py release --stage <stage> --outdir <dir>   # unmounts through sudo; also a bare mount point
python3 tools/trace_stage.py abort --stage <stage>                    # only a stage without data
```

Two read-only diagnostics come with it (approved 2026-11-09), as context,
never as a cause by themselves: `tools/vm_counters.py` brackets every probe
with the system-wide swap, major-fault, direct-reclaim, `allocstall_*` and
writeback counters and the `MemAvailable`, Dirty, Writeback, SwapFree and
Shmem gauges (raw values, deltas with units, unavailable values marked), and
`tools/trace_correlate.py` lists every recorder's off-CPU span over 5 ms that
began with a switch-out in D state, from the trace's own events, split at
the recorder's `sched_wakeup` into the time blocked and the time runnable
until it ran again (a span whose wakeup the trace lacks is not split).

The implementation's review (2026-11-09) found two blockers, both
reproduced without root, and a diagnostic's wording to correct: the mount
point, created by root with mode 0700, refused the operator once unmounted,
so a successful release failed its cleanup and left a stage that blocked
the next session; a 10 s untraced stage at 1000 Hz got 45 MiB under a
64 MiB guard bound, so it tripped empty; and the recorders' spans, measured
to the next switch-in, were called uninterruptible though they could
include a runnable wait. The fixes are above: the mount point is the
operator's and is never entered after the unmount, the stage holds the
guard's reserve, and the spans are split at the wakeup. The stand-ins now
emulate a real unmount (the contents vanish and the mount point reappears
with its own mode) and the stage's bounded capacity. The fixes were
confirmed the same day, and the reviewer approved S with its reserve.

**Rehearsals 5 and 6 (2026-11-09; not accepted).** At `b6d9fde`, archived in
`stage1_rehearsal/` (`stage1-controls-5`, `stage1-placement-6`). No controls
workload or timer-mode probe ran: the controls and the open timer run stopped
before them at the buffers' readback, 524,384 KiB per array against an
allowance of 524,288 that ignored the kernel's rounding (16 MiB per CPU is
4,113 subbuffers of 4,080 bytes, 16,387 KiB). Their startup traces (3.3 and
4.1 MB) were published. The open untraced control ran on its 109 MiB stage
(the guard's sampled peak 1.8 MB, no stop) and fails T1, T2 and T6a, as every
untraced emulator run has. All three stages were real `noswap` tmpfs mounts
of exactly the requested size, the operator's with mode 0700, published with
verified ledgers, unmounted and removed; no confined run started. This
validates the staging lifecycle and a short untraced run, not sustained
recording without losses, complete timing and SMI coverage, or the confined
placement. The readback now follows the kernel's rounding (above; approved
2026-11-09). Its implementation was confirmed the same day, and both
rehearsals repeated as attempts 7 and 8.

**Rehearsals 7 and 8 (2026-11-09).** At `8318db6`, archived in
`stage1_rehearsal/` (`stage1-controls-7`, `stage1-placement-8`, with their
analysis reports). Attempt 7 is accepted for the recording and controls
checks (reviewer, 2026-11-09). The controls pass (positive and both
negatives). Its timer run has zero main-trace and TSC-instance losses and
zero packets reported dropped by tcpdump; its 9,465 sleeps were all on time,
U is 2,020 ns, and the SMI counts cover the run. Its analysis fails only the
expected PTY USB association, and its untraced control fails T1, T2 and T6a,
as every emulator run has. The root-only geometry and readback checks
worked: every trace array read back exactly. No recorder off-CPU span over
5 ms began in D state during the probe windows (tcpdump's came before
them). The controls count toward acceptance while the kernel, the probe
definitions and the relevant events stay unchanged. Attempt 8's open timer
run is as complete (its 9,460 paired sleeps on time, one cycle having begun
without sleeping; U 1,920 ns), but its confined untraced control was a
placement failure. Halfway through it, a thread of a Claude Code process
(`JITWorker`), absent from the placement record, was allowed on every CPU,
and the session stopped before the confined timer run. Confine had listed
that process's threads, then changed them; the thread is consistent with
being created in between, though the untraced record does not date its
creation. The placement rehearsal is not accepted. Confine now repeats its
pass, and the checks follow the observers' lineage (above; approved
2026-11-09). Its review found unreadable identities and task directories
taken for exits, in confine, restore and the task listing, and the halfway
snapshot's status discarded; they now fail, and the ancestry limitation
above is recorded (review, 2026-11-09). The correction was confirmed the
same day, and the open and confined placement session repeated as attempt 9
in a fresh directory, with freshly collected observer PIDs and the emulator
prepared as above:

```
tools/quiet_host.sh stop
tools/trace_session.sh --port /tmp/rtctrl-rehearsal-tty --usb-device /sys/bus/usb/devices/1-9 \
  --modes untraced,timer --placements open,confined --observers <PID,...> \
  --runs current:1000:rt-idle:fast-motion --duration 10 --attempt 9 --outdir <dir9>
tools/quiet_host.sh restore   # also after a failed or interrupted session
```

**Rehearsal 9 (2026-11-09; accepted).** At `6933d33`, archived in
`stage1_rehearsal/stage1-placement-9` with its two analysis reports. It is
accepted as the emulator recording and observer placement rehearsal, within
the documented ancestry limits (reviewer, 2026-11-09). Attempt 7's controls
still count while the kernel, the probe definitions and the event set stay
unchanged.

| Run | Placement | Recording |
|---|---|---|
| open, untraced | (observers left as they are) | (untraced) |
| open, timer | (observers left as they are) | 0 main-trace and TSC-instance losses, 0 packets dropped by tcpdump; 9,464 paired sleeps on time; U 2,181 ns, one suspect anchor excluded under the existing rule (19,654 of 19,655 matched, skew -2,183 ns); SMI counts cover the run, 0 SMIs |
| confined, untraced | 1,367 threads in 4 passes, 1,360 masks changed; midpoint snapshot complete, none of 1,398 observer threads and neither of 2 probe helpers allowed on CPUs 2 or 3 | (untraced) |
| confined, timer | 1,371 threads in 2 passes; midpoint snapshot, none of 1,361 observer threads or 2 probe helpers allowed there; trace: no time on CPUs 2 and 3 by attributable observer tasks (1,370 in the lineage listed after the trace started, none absent from the record, 39 created during the trace) or probe helpers | 0 losses as above; 9,468 paired sleeps, one classified as interrupt late from idle state 1; U 2,097 ns, all 19,652 anchors matched; SMI counts cover the run, 0 SMIs |

In the confined untraced run, pass 2 found a `JSCWarmUp` thread of a Claude
Code process still on the excluded CPUs and confined it; its creation in pass
1's gap is an inference, as the record does not date it. Pass 3 found seven
`HeapHelper` threads already off them (inherited), and pass 4 settled. The
late service's interrupt arrived about 111.5 us after its converted deadline:
this locates the delay, not its cause, and does not show that the idle state
caused it. The confined run's two worker waits over 50 us (86.8 and 76.7 us)
start at `sched_waking`; the completed `sched_wakeup` came 85.8 and 75.7 us
later, about 1 us before the switch-in. They were mostly interrupt-context
spans before the wakeup completed, not a runnable FIFO worker kept waiting
by other tasks, and neither timer run recorded a fair-server selection.
Neither analysis has a probe miss or a NULL-page fault, and both fail only
the expected PTY USB association; all four runs fail T1, T2 and T6a, as
every emulator run has. All four stages were published with verified ledgers
and released, both restorations finished with no thread left or failed, and
the host settings were restored.

The untraced placement evidence is one midpoint snapshot. The timer run's
evidence covers the tasks whose observer lineage could be established,
subject to the ancestry limitation above; it is no proof that every possible
descendant was isolated. The emulator's expected timing and USB failures
qualify no hardware control rate. Hardware diagnostic runs wait for the
review of their concrete run plan.

**Stage 1 hardware sitting (plan approved 2026-11-10).** The concrete run
plan was reviewed four times on 2026-11-09 and 2026-11-10 and accepted at its
revision 54. It runs torque-off with the owner physically at the robot: every
servo torque-off, the arm at rest and supported, the actuator power cutoff
within reach and the servo power on. It approves no remote execution and no
torque-on, and it qualifies no control rate. Every run is 20 s at attempt 2's
condition (current profile, 1000 Hz, `rt-loaded-excl`, Fast Sync Read with
the motion-only read, SCHED_FIFO 80 on CPU 3), in ten invocations (nine
without H3), each reviewed before the next probe:

| # | Session | Attempt | Runs | Directory under `~/rtctrl-hw/` |
|---|---|---|---|---|
| 1 to 3 | H1, mechanism reproduction | 3 to 5 | untraced, timer (observers open) | `stage1-mechanism-3` to `-5` |
| 4 | H2, placement comparison, open first | 6 | open untraced, open timer | `stage1-placement-6-open` |
| 5, 6 | H2 | 6 | confined untraced, then confined timer | `stage1-placement-6-confined-untraced`, `-timer` |
| 7, 8 | H2, confined first | 7 | confined untraced, then confined timer | `stage1-placement-7-confined-untraced`, `-timer` |
| 9 | H2 | 7 | open untraced, open timer | `stage1-placement-7-open` |
| 10 | H3, only after valid H1 and H2 | 8 | untraced, timer at 500 Hz, `rt-loaded` | `stage1-500hz-8` |

Each confined run is an invocation of its own, so its restoration is checked
before any later probe; each H2 invocation keeps its attempt and label. The
invocations go through an operator procedure kept outside the repository,
`~/rtctrl-hw/stage1-procedure.bash` (400 lines, SHA-256
`5a6ac9884a8b5af0060ef5041d1db852d62321854b8d6aff6f98b244410a7619`, installed
read-only on 2026-11-10 and its hash verified; it replaced the first
sitting's 397-line `2f91431f11070aa204538b4cd089ddb54f19a712a8ebd09bf1558f14b029ad0c`,
kept beside it as `stage1-procedure-rev54-2f91431f.bash`). The procedure:

- records the identity at execution (the commit, any tracked change, the
  kernel, the diff since `8318db6` of the probe, its definitions and the
  timer analysis, and the probe binary's and its own SHA-256), and runs no
  invocation until the developer has reviewed that record by its SHA-256;
- runs each invocation in a bash of its own, started without `BASH_ENV` and
  with every signal defaulted, which refuses to start when a latched signal
  is not caught or is blocked;
- stops the resident services with `tools/quiet_host.sh`, owes their
  restoration from then on and makes it after the session, a failure or an
  interruption; on a fault signal it restores only once the running step's
  end is confirmed, and otherwise leaves the host as it is and reports the
  recovery;
- latches every catchable signal that would end it as `STOP`, letting the
  running step finish first (Ctrl-C reaches the session's own cleanup);
- checks the session's and the restoration's statuses, the stage root, the
  planned run records and timer runs (a timer run is a run record, so a
  run's own `.csv.smi.csv` and `.csv.tsc-pairs.csv` are not counted), each
  stage's release and recorded cleanup, and for a confined run a complete
  restoration with no thread left at its mask;
- analyses each timer run with `tools/trace_correlate.py` only after those
  checks pass;
- latches any `STOP` in the calling shell and records it beside the identity
  record, and refuses every later invocation of the sitting.

Its 82 stand-in cases and the reviewers' reproducers pass, and the accepted
emulator rehearsals needed no repeat for these procedure-only changes. Any
`STOP` or failed validity check ends the sitting with no retry and no changed
setting, limit or criterion, and goes to the reviewer; so does a fault-signal
exit whose recovery stayed unresolved, and SIGKILL or a power loss. On the
reviewer's approval, each invocation's files go to a new `stage1_hardware/`
section of the private archive with their analysis reports, the identity
record, any stop record and the procedure file.

**H1 attempt 3 (2026-11-10; accepted).** The first sitting began at
`b3979b0` with the 397-line procedure, its identity record reviewed.
Invocation 1 (`stage1-mechanism-3`) ran both runs (`trace_session` exit 0):
both stages were published with verified ledgers and released, the stage
root was left empty and the host settings were restored. The invocation then
stopped on a false count, "3 timer runs ..., 1 planned": its `*-timer-*.csv`
pattern also matched the timer run's own `.csv.smi.csv` and
`.csv.tsc-pairs.csv`, which its stand-in tests had never written. That STOP
ended the sitting, and it stays ended (its stop record kept). The timer
run's analysis was run by hand afterwards, outside the capture directory,
and copied into it after review as derived evidence with a provenance note
(its SHA-256, the analysis revision at `b3979b0`, and the fact that it ran
after the STOP); the capture's ledgers are its own. The reviewer accepted the
diagnosis and the corrected procedure, and counted attempt 3 as H1's first
attempt, not to be repeated (2026-11-10):

| Run | Recording | Timer service |
|---|---|---|
| untraced | (untraced); 13,216 cycles, mean 660.8 Hz, 0 read and 0 write failures | (untraced) |
| timer | 0 main-trace, TSC-instance and tcpdump losses; SMI counts cover the run, 0 SMIs; 14,491 cycles, mean 724.5 Hz, 0 read and 0 write failures | 14,489 paired sleeps, two served late as interrupt late from idle state 1 (70.3 and 57.5 us); U 2,035 ns, all 40,700 anchors matched; no fair-server selection; the worker's waits after waking on CPU 3 at most 20.0 us |

Both runs fail T1, T2 and T6a, as diagnostic results. No fair-server
selection in this capture is valid negative evidence, not proof that the
mechanism never occurs; the two late services locate interrupt delays and
establish no idle-state cause. One untraced and timer pair does not show that
tracing improves throughput. H1 resumes at attempt 4 in a new sitting, with a
new shell and a new identity record, and the remaining order unchanged.

**The sudo refresh and its rehearsal (attempts 10 and 11, 2026-11-10).** The
second sitting ran H1 attempts 4 and 5, both accepted, then H2 attempt 6's
open pair. That timer run failed its validity check with two kernel page
faults at NULL-page addresses on CPU 3: both fall in one `sudo` started by the
session's sudo refresh, unpinned at the time, inside its `statx` call with a
NULL path. The failed verdict stands, the capture does not count toward H2 and
the full open pair is repeated. H2 restarts as attempts 7 (open first) and 8
(confined first), and H3 becomes attempt 9, which supersedes the table's labels
above (directories `stage1-placement-7-open`, `-7-confined-untraced`,
`-7-confined-timer`, `-8-confined-untraced`, `-8-confined-timer`, `-8-open` and
`stage1-500hz-9`).

The refresh now pins itself off the excluded CPUs (`96e79a3`), its end is
confirmed (`d710b9f`) and `tools/refresh_check.py` (`e416178`) checks its
placement in a timer run. The review of those two found four defects,
corrected and approved: `fb0d514` (the checker requires complete, well-formed
evidence and checks every task) and `53386ed` (the stop tells ended, running
and not known apart, and requires a valid ledger). `25eb359`, retained by the
reviewer, repeats the stop request at each poll inside the same polling limit:
a request can go unanswered (the signal is sent and USR1 is caught, but the
trap does not run), the cause is unknown, and an unconfirmed stop must still
fail.

Both rehearsals ran at `25eb359` on Linux 7.0.0-42-generic #42, over `dxl_emu`,
50 s, `rt-idle`, timer mode, observers open, and are archived in
`stage1_rehearsal/`:

| Attempt | Verdict | Refresh and placement | Timer conversion |
|---|---|---|---|
| 10 | failed as a complete rehearsal | one refresh 47.52 s into the run; its four attributable tasks only on CPUs 22, 27 and 28; no CPU 3 NULL-page fault | the matcher could not establish an acceptable match at anchor 44370; the accepted span ends about 22.75 s into the run, before the refresh, and U = 1,761 ns applies only to it |
| 11 | accepted within its stated limits | one refresh 48.37 s into the run; its four attributable tasks only on CPUs 4, 20, 25 and 28; no CPU 3 NULL-page fault | all 97,556 instance anchors paired with no suspects; the run and the refresh covered; U = 1,914 ns |

At attempt 10's anchor 44369 the incoming interval deviates by about
-2,018 ns against a 2,099 ns tolerance, and the outgoing one by about
+2,023 ns against 2,001 ns, failing by about 22 ns; the next interval agrees,
so the isolated-suspect rule rejects it. Equal anchor counts and continued
alignment support a boundary-case explanation but do not establish an
acceptable match under the approved rule, and the capture establishes no
recurrence probability. The margin is no reason to waive or round a
threshold, and `tools/timer_service.py` is unchanged; a future change to which
anchors may be excluded would be an evidence-rule change, with its own design,
regression review and controls validation. A what-if that removes that anchor
is kept apart as diagnostic work, never as acceptance evidence. Attempt 11 was
the one further attempt the reviewer explicitly authorized, with the code,
kernel, duration, event set, thresholds and rules unchanged, and with its
output directory created first, by a creation that refuses an existing
directory, before any quiet-host change (attempt 10's first invocation had
refused for a missing directory, before any capture).

Attempt 11 passes the stated requirements within the recorded limits. The only
failure of the shared analysis is the emulator's PTY USB association, an
exception that cannot be used on hardware; it has zero losses, 5,102 SMI
intervals covered, and verified publication and cleanup. Its five late sleeps
are classified as interrupt late from idle (about 56.7 to 121.8 us), a class
that locates the delay and establishes no cause. Of its two worker waits of
50 us or more, classified as other mechanism with no recorded fair-server
selection, the 124.6 us one is mostly recorded interrupt time and the
104.4 us one mostly recorded idle and switching time. T1, T2 and T6a remain
failed, and no hardware control rate is qualified. The checker proves
placement only for attributable tasks from its listings and fork events: the
pre-run listing is per process, so it does not prove that every possible
descendant, or every pre-existing thread that vanished before the midpoint,
was isolated. Cleanup statements cover the session's tools and the refresh's
ownership, not every unrelated host process.

An `apt upgrade` on 2026-11-10 installed 7.0.0-43, on which `rq.ext_server`
and `rq.fair_server` move by 8 bytes and three operands of `rtfs_dl_pick` with
them: a layout change, not a shown change of behaviour. The reviewer and the
owner returned to 7.0.0-42 for attempts 10 and 11 and the remaining H2 runs
(the default boot entry, the versioned packages marked manual); after the
reboot the kernel line, probe layout, eight definitions and boot line matched
the records. H1 and the controls stay accepted as 7.0.0-42 evidence and do not
qualify 7.0.0-43. The old kernel does not restore the rest of that upgrade, and
the resumed sitting's provenance records it: `linux-perf` 7.0.0-42.42 to
7.0.0-43.43, `bpftool` 7.7.0+7.0.0-42.42 to 7.7.0+7.0.0-43.43,
`google-chrome-stable` 156.0.8078.47-1 to 157.0.8103.50-1 and `rtkit` 0.14-1
to 0.14-1ubuntu0.1. Hardware resumes only after the final plan and a fresh
execution identity are reviewed.

**Sitting 3: H2 attempt 7, and the pause (2026-11-10).** The final plan
(revision 81) and a fresh execution identity at `5adb1a5` were reviewed, and H2
attempt 7 ran three invocations, each reviewed before the next:

| Invocation | Verdict | Evidence |
|---|---|---|
| open pair | accepted as valid diagnostic evidence | USB association PASS (25,886 transfers); all 41,620 instance anchors matched with no suspects; U = 1,873 ns; no losses, no SMIs, no NULL-page faults |
| confined untraced | accepted as independent diagnostic evidence, within its midpoint-snapshot and ancestry limits | 1,198 threads confined in 2 passes with no failure; at the midpoint none of 1,172 observer threads or 2 helpers allowed on CPUs 2 and 3; restoration left none behind |
| confined timer | failed validity check 3, not accepted; its STOP ended the sitting | the matcher could not establish an acceptable match at anchor 22810, whose continuation passes the same interval test at offsets -2, 0 and +2. The accepted span ends 14.07 s into the run, and U = 1,668 ns applies only to it |

The confined timer run's USB association (26,490 submissions for 26,490
transfers), placement (no attributable observer or helper time on CPUs 2 and 3)
and cleanup subchecks passed and remain diagnostic evidence only. Equal or
compatible anchor counts, zero losses and a low uncertainty for the accepted
prefix do not repair the missing full-run coverage. This differs from attempt
10's unequal-tolerance boundary case; the two runs are two of the last four
timer acquisitions, which is a small selected set and establishes no recurrence
probability.

Normal-priority scheduled residency on CPU 3 derived from `sched_switch`, inside
the run window, was 101.9 ms in the open timer run and 285.4 ms in the failed
confined one. These figures do not establish that confinement worsened
performance, and they cover the tasks attributable through the saved lineage
and forks; no observer-set change was made during the series.

H2 is incomplete, and no retry, attempt 8 or H3 is cleared. Hardware is paused
for an offline clock-alignment and anchor-exclusion design, reviewed before the
production acceptance rule changes. It needs a general criterion distinguishing
an isolated stamp disagreement from missing or duplicated events and unresolved
periodic aliases, with tolerances from an independently justified error budget.
Its validation is planned as frozen examples of both observed failures, negative
synthetic cases, a replay over archived captures with original verdicts
preserved, then the emulator controls and end-to-end timer validation on
7.0.0-42 and a revised H2 plan with fresh labels and identity. The invocations
are archived in `stage1_hardware/`, each with its own verdict.

That design was frozen as revision 53 on 2026-11-11, and the session and the
analysis now implement it (the conversion and the timekeeping record, above).
The implementation review of 2026-11-11 requested three corrections, which its
re-review accepted: the cleanup's existence checks answer only from a listing
read in full, the evidence files are read strictly, and the audited trace-cmd
build is enforced for every use. The re-review then required the trace
libraries and the plugin set to be pinned as well. Its review accepted the pins
and required the launch's producer to be checked by its status and whole
output; that correction was approved (`2470255`), and the implementation was
cleared for the emulator validation. Changing `tools/timer_service.py`
voids the controls accepted under the earlier analysis for new work; their
recorded results stand.

**Emulator validation (2026-11-11; partly accepted).** Two sessions over
`dxl_emu` at `2470255` on 7.0.0-42, archived in `clock_validation/`: attempt
12, fresh fair-server controls on CPU 5 and an untraced control and a timer
run at current, 1000 Hz, `rt-loaded-excl`, 20 s; attempt 13, an untraced
control and a timer run at `rt-idle`, 50 s, attempt 11's condition. The
reviewer split the verdicts:

| Evidence | Verdict |
| --- | --- |
| Attempt 12's controls | Accepted as fresh controls under the recorded kernel, probe definitions, event set and tool conditions |
| Attempt 12's recorder, extraction, report, certificate and conversion under load | Accepted as a component demonstration: CERTIFIED, the clock rule passes, U about 1,098 ns |
| Attempt 12 as a complete validation | Not passed: the analysis fails on 105 worker timer wakes outside every sleep, beyond the PTY USB exception, and both probe runs fail T3 (14 and 3 read failures) |
| Attempt 13's certificate | Valid, and separate from the anchor accounting |
| Attempt 13's conversion | Rejected: one local timer entry with no deadline of its own fails the deadline-slot accounting; the frozen rule and the failure stand, and 524 ns is not an accepted bound |

The 105 wakes lie inside the three failed-read write calls, 35 each: the
port waits out its guard after a read timeout in sleeps of at most 1 ms. The
generic check could not account for those sleeps. The accounting review of
2026-11-11 confirmed, by its own replay, that these 105 are guard sleeps,
including the last slice of each guard expiring 617 to 973 ns after the
guard's end (within the 1,422 to 1,851 ns from the previous sleep's return,
inside the same write call, to the arming). It found revision 2 of the
generic rule too loose: it accepted a sleep call made before the write call,
and let a return from before the write call bound the lag.

Wake accounting revision 3 (`tools/trace_correlate.py`) accounts a wake as a
guard sleep only if the failed read's write call owns it:

- The sleep's call, the worker's arming of its timer (at most 100 us later),
  the wake and the sleep's return lie in that order inside the write call.
  The call is part of that write call's complete sequence of sleeps: no
  return before its first call, and exactly one return per call.
- The guard was still pending when the sleep read its clock. The sleep's
  floor (the write call's start for the first sleep, else the previous
  sleep's return) is before t0 + budget + guard.
- The sleep lasts at most 1 ms, from at or after t0 + budget. Its expiry is
  at or before t0 + budget + guard plus the time from its floor to its
  arming; a return from before the write call bounds nothing.

Every other wake stays a failure. The tests include the review's two
counterexamples, each refused. Revision 3 still accounts all 105 of attempt
12's wakes and leaves none unexplained. Its T3 failures stand, and their
cause is not established; the read timeouts also occur in the untraced run.

The first cost figures were not bounds. A difference of conditional medians
can understate the recorder's contribution or be negative, and those runs
mixed updates from outside the measured window: attempt 12's two CPU 3
updates came before its run. `tools/tk_cost.py` restricts everything to one
recorded window, the probe's first wake to its last cycle end, and reports
those differences as descriptive (accepted by the accounting review). Its
cycle wakes are the sleeps the trace establishes, through the same pairing
as the timer-wake check, since a deadline still ahead at a cycle's end stamp
does not show a sleep: the probe's bookkeeping after the stamp can use up
the rest of the period. Inside the windows:

- Attempt 12: CPU 3 performed no update; CPU 5 performed 19,617, at a median
  of 0.49 us each. The trace establishes 18,029 sleeps; one transition began
  awake.
- Attempt 13: CPU 3 performed 47,597, at a median of 0.32 us each. The trace
  establishes 47,323 sleeps; 32 transitions began awake.
- No late sleep wake and no worker wait of 50 us or more overlapped an
  update on CPU 3.

These are observations, not cost limits. A U whose clock rule failed is now
printed as a candidate, "candidate U; conversion not accepted", never as
validated; the certificate's verdict stays separate. This is a reporting
change only, approved by the accounting review, with the numerical rule
unchanged. Attempt 13 prints its 524 ns that way. No new acquisition was
needed for these corrections.

The re-review (2026-11-11, at `177faee`) accepted the three fixes and closed
both accounting findings. Revision 3 refuses the counterexamples and still
accounts attempt 12's 105 guard wakes, and the cost tool's sleeps are the
trace's. What stands:

- Attempt 12's fresh controls and its recorder-to-conversion component
  demonstration remain accepted under their recorded conditions.
- Attempt 12's T3 still fails (14 untraced and 3 timer read failures, cause
  not established). Accounting for the guard sleeps neither waives those
  failures nor shows whether the recorder caused them, so the overall
  validation has not passed.
- Attempt 13's certificate is valid and its conversion rejected; 524 ns is a
  candidate, not an accepted bound.
- The cost observations stay descriptive.

**T3 diagnosis (2026-11-11; accepted by the reviewer, with an independent
replay).** In attempt 12's timer run, all three failed reads follow one
chain, and the eight successful reads slower than 10 ms the same one:

1. The worker's instruction write on CPU 3 queues the pty's tty flush work
   (`flush_to_ldisc`, on an unbound workqueue) and wakes its kernel worker,
   a normal-priority thread, onto CPU 3.
2. The worker then busy-polls `read()` at SCHED_FIFO 80 for the whole 35 ms
   budget: tens of thousands of zero-byte reads and no switch on CPU 3, so
   the kernel worker cannot run. The emulator's read blocks in the tty
   layer waiting for that flush (seen in the first failure).
3. CPU 2, the only other unloaded CPU, was running a non-idle task at each
   budget's start and entered idle at no point inside the budget, so
   nothing moved the kernel worker away.
4. When the read times out and the worker sleeps in the guard (+35.02 ms),
   the flush runs at once on CPU 3, and the emulator's reply follows too
   late.

In the slow successful reads the flush ran on CPU 2, 10 to 29 ms after the
instruction, after CPU 2's idle context moved the kernel worker there from
CPU 3. In the probe logs, a failed read without `first_reply_ns` means that
no reply section was completed and parsed, not necessarily that no byte
arrived. The trace establishes zero-byte reads for the three traced failures
only; the 14 untraced failures share the timeout signature but have no
trace. The recorder is not needed for this failure family (the untraced run
failed without it), which does not show that it had no influence. In the
archive, attempt 12 is the first emulator run on the bounded read path under
load; the 22 earlier emulator runs (all idle) had no timeout, and neither did
the 21 hardware runs under the same load. On hardware the instruction is
sent as a USB transfer without a send-side flush, but the receive side also
depends on that normal-priority work, so the same starvation remains a
design concern there; a finite timeout-free sample shows no immunity.

**Option A: the loaded validation with CPU 3 out of the unbound workqueue
mask (procedure for review; not yet executed).** The reviewer chose a fresh
loaded emulator validation with CPU 3 cleared from the kernel's global mask
for unbound workqueues (`/sys/devices/virtual/workqueue/cpumask`, `ffffffff`
on this host), every budget unchanged. The mask governs every unbound
workqueue, not only the tty flush, so it is a changed host condition,
recorded as such, and never carried into hardware without a separately
reviewed plan. A pass supports the diagnosis and supplies a fresh end-to-end
emulator pass under this declared host policy; it does not pass attempt 12
or qualify a control rate. A failure shows that this intervention is
insufficient, without erasing what the three traces established. No idle
pass is required besides it.

The first version of the procedure (`d9e774e`) was not ready: its first
session command was refused (the controls need the timer mode), nothing
owned the restoration across a hangup, and the new checks had false passes.
The second (`04e8822`) still restored while a process of the experiment might
be running, let the keeper's replies follow a link the invoking user planted,
could start a step after a latched stop or a record that failed, and read a
field given twice by its last value. The third (`7553f40`) still confirmed
the end of the experiment's processes by a `/proc` scan, which a process
that forks and then exits during the scan can escape, and let the keeper's
status files take the caller's umask (unreadable to the user when root owns
them). The fourth (`2111f92`) let a release that failed fall through to the
keeper's group-only fallback, which could restore the mask while a
descendant in a session of its own still ran. The corrected procedure is one
supervised command, and these tools serve it:

- `tools/option_a.sh` is the supervisor. Before any host change it refuses
  with nothing changed unless the kernel is 7.0.0-42, the emulator's pty
  exists, the real-time limit allows the probe's priority, no mask or
  quiet-host record is open, and the three directories do not exist. It
  then creates them, fresh, and requires both session commands to pass the
  session's dry run, the operator's confirmation, sudo and a ready keeper.
  Then, recording each step's start, end and status, and each obligation
  before the change that creates it, it runs: the quieting, the mask's
  clear, the untraced session (attempt 14), the mask check and the gate,
  then the controls with the timer session (attempt 15), the mask check and
  the gate. Before each step it checks for a latched stop and records the
  step's start; a stop, or a record that cannot be written, ends it before
  the step begins. Any nonzero status or a signal stops it with no further
  step. It runs as the child subreaper of everything it starts
  (`PR_SET_CHILD_SUBREAPER`, set through perl, which leaves every signal's
  disposition unchanged, before it execs itself again): a descendant whose
  parent ends is reparented to it rather than to init, whatever its process
  group or session; nothing else about the steps changes. Its cleanup first
  needs a confirmed end of every other process it started: the kernel's
  list of its children holds nothing but the keeper. Every running
  descendant then has an ancestor in that list (a zombie counts until it is
  reaped), the kernel produces the list in one pass, and no new descendant
  can appear once it is empty, so a fork during the check cannot escape it;
  a list that cannot be read is no end. It waits up to 120 s for that.
  Without it, it changes nothing on the host, names what remained, releases
  the keeper (which then ends without restoring) and ends unresolved, for
  the operator to restore by hand once those processes have ended. It
  records whether the keeper confirmed the release, had acted on it although
  its answer was lost, or still runs (a release never delivered); in each
  case nothing is restored, since the keeper never restores on its own. With it,
  it restores and verifies the mask and the quieted services, then ends the
  keeper.
- `tools/root_keeper.sh` is the privileged half, started through sudo while
  the terminal is alive (sudo-rs ties the cached credentials to the
  terminal, so sudo may refuse after a hangup). It writes only the original
  mask or the target, and stops or starts only `unattended-upgrades`; it
  refuses anything else. Its directory, `/run/rtctrl-keeper-<pid>-<start>`,
  is created by root exclusively (mode 0755), so the invoking user writes
  only the request FIFO in it; every file the keeper writes there is created
  exclusively, without following a link, relative to that directory, given
  mode 0644 explicitly (whatever the umask, which sudo may pass on), then
  renamed into place, and nothing but the FIFO changes owner. It never
  restores anything on its own. If its owner ends without a finish or a
  release (the supervisor was killed, crashed, or its release never
  arrived), the keeper ends WITHOUT restoring, and logs the state of the
  supervisor's process group from the kernel (`kill(-pgid, 0)`, which a
  fork cannot race; for the operator's information only, since it does not
  reach a descendant outside the group). An owner that cannot be read is not
  taken as ended.
- `tools/wq_mask.sh` changes and restores the mask with its append-only
  record. Its census of the unbound workers and rescuers counts an entry
  that still exists but cannot be read, or has no well-formed affinity list
  (a descending range such as `3-1` included), as unknown; an unknown entry
  fails the verification and the restoration.
- `tools/validation_gate.py` checks one finished run. Completeness and T3
  come from `tools/timing_summary.py` and must agree with the session's
  record, the failure counters must be zero, and the verdict may fail only
  T1, T2 and T6a, on the emulator's pty. It reads the mask record as a checked
  sequence: the fixed values once each, CPU 3 cleared from equal originals,
  then the actual masks and counts of every step in order. Each step holds
  its own fields once each: a field given twice, or an empty, missing or
  extra field, makes the record unreadable (exit 2), as it does in the
  experiment record's session markers; a reason is free text. It bounds the run
  by the supervisor's start and end of its session (the session's own note
  is written before the load and recorders start). It runs with the record
  open between the runs, and closed after the experiment.
- `tools/unbound_work.py` checks the timer capture. CPU 3's occupants come
  from its switches, from the one running when the window began; no unbound
  worker or rescuer may run there inside the window, and no unbound work or
  tty flush may execute there, including work begun before the window. Every
  instruction flush must be seen to run, and none on CPU 3. A malformed
  event, a line that is no event, broken switches, a short report or a
  decoder that fails, even after its last useful line, make the evidence
  unreadable.

The operator, at the workstation, from the repository root, on the emulator
only, with the arm powered off (or torque-off and supported) and nothing sent
to it:

0. Preconditions, as for attempts 12 and 13: `dxl_emu` serves
   `/tmp/rtctrl-rehearsal-tty`; each emulated servo's return delay is 0
   (`./build/apps/dxl_inspect --port /tmp/rtctrl-rehearsal-tty write <id>
   return_delay_time 0` for ids 2 to 9); `sudo prlimit --pid $$
   --rtprio=90:90` in the shell that runs the supervisor. Running it inside
   `tmux` keeps a closed terminal window from hanging it up.
1. `tools/option_a.sh --dry-run`, then `echo "exit $?"`: exit 0 shows both
   session commands accepted, with nothing changed.
2. `tools/option_a.sh`, then `echo "exit $?"`. It asks for the confirmation
   and for sudo once. The sessions run with `--yes`; their commands are fixed
   in the script (as they were for the first run, attempts 14 and 15; the
   second run's differences are below):

    ```sh
    tools/trace_session.sh --yes --port /tmp/rtctrl-rehearsal-tty --usb-device /sys/bus/usb/devices/1-9 \
      --placements open --runs current:1000:rt-loaded-excl:fast-motion --duration 20 \
      --modes untraced --attempt 14 --outdir ~/rtctrl-hw/clock-validation-14
    tools/trace_session.sh --yes --port /tmp/rtctrl-rehearsal-tty --usb-device /sys/bus/usb/devices/1-9 \
      --placements open --runs current:1000:rt-loaded-excl:fast-motion --duration 20 \
      --fs-controls 5 --modes timer --attempt 15 --outdir ~/rtctrl-hw/clock-validation-15
    ```

Its exit status:

- 0: both runs were acquired, both gates passed, and every restoration was
  verified. The acceptance is then judged as below.
- 1: it stopped after a failed step, a failed gate (the untraced run's T3
  above all: no timer run) or a signal, with every restoration verified.
- 2: it refused, and nothing changed.
- 3: unresolved. The end of the experiment's processes was not confirmed
  (nothing was restored, and the keeper, released or not, restores nothing
  either),
  an owed restoration was not verified, or a record could not be
  written (the final one included: the outcome is then never 0). The
  supervisor names each unresolved item and its remedy. Stop, keep the
  records, and restore (`tools/wq_mask.sh restore`,
  `tools/quiet_host.sh restore`) before anything else; neither tool starts
  beside an open record. A reboot resets the kernel's mask, after which
  `restore` verifies and closes the record.

Before the first acquisition, the keeper was checked on the real host, with
no change: on 2026-11-12, at `03b6ba2` (after its final preflight passed),
the operator started it as root through sudo-rs 0.2.13, read its status
against the recorded masks and finished it (a finish sent only if every
earlier check passed and the masks still read as recorded). Its 18 checks
passed: the keeper ran as root; its directory and replies directory (0755)
and its ready file, log and reply files (0644) were root's, its FIFO the
user's (0600);
the masks read `ffffffff` before, during and after; it ended, its FIFO
removed, and its log records no write, mask request or service operation.
This result is for review; Option A's acquisition waits for it.

**Attempts 14 and 15 (2026-11-12, at `87d39d8`, after that check was
accepted): stopped by the timer run's gate.** The supervisor cleared CPU 3
from the mask (45 workers, none allowed on CPU 3). The untraced run
(attempt 14) had no bounded-read timeout: T3 passed, and its gate passed 14
of 14 checks. The fair-server controls passed. The timer run (attempt 15)
had one timeout (cycle 17621, 19.25 s into the run, no completed parsed
reply section within the 35 ms budget, the next reads recovered): T3
failed, and its gate passed 11 of 15 checks. The supervisor stopped (exit
1), restored and verified the mask and the quieted services, and finished
the keeper. On attempt 15's capture the residency check passes: no unbound
worker or rescuer ran on CPU 3 in the window, and all 36204 instruction
flushes ran, none on CPU 3. The validation has not passed: the acceptance
review accepted attempt 14 as the untraced component result and found
attempt 15 and the overall validation failed.

**Attempt 15's timeout (offline diagnosis, accepted).** The trace establishes
emulator starvation as the timeout's immediate mechanism. Between the
worker's cycles, CPU 3's idle context migrated the runnable emulator
(`dxl_emu`, an ordinary task allowed on every CPU) onto CPU 3; its poll
timer, armed there, woke it onto CPU 3 0.80 ms after the instruction, whose
flush had already run on CPU 2. It then waited 34.22 ms, runnable, behind the
busy-polling `SCHED_FIFO` worker, with no switch on CPU 3 and no idle entry
on CPU 2, and read the instruction and replied only after the read's budget
had ended. The two longest successful reads (29.8 and 11.8 ms) show the same
wait, ended by a migration back to CPU 2. The mask did what it governs; the
emulator lies outside it. Why the scheduler chose those placements, the
recorder's influence and the milder slow reads are not established. The
certificate (U 1373 ns accepted) and the wake accounting (every extra wake a
guard sleep of the failed read) stand as component results; T3 and the
overall validation stay failed.

**Option A, second run: the emulator kept off CPU 3 (procedure for review;
not yet executed).** The emulator is launched with an affinity that excludes
CPU 3, everything else as in the first run: the workqueue exclusion, the
stress placement, the durations, the 35 ms budget and guard, the recorders,
the gate and the acceptance rules, and the stop without retry. The changed
experimental conditions are the unbound workqueue mask and the emulator's
affinity, both recorded. Keeping the emulator off CPU 3 is a testable
prevention of the co-location attempt 15 showed; it does not guarantee zero
timeouts, and it qualifies neither the real USB path nor 1 kHz operation.

- `tools/option_a.sh` runs attempts 16 (untraced) and 17 (timer), into
  `clock-validation-16`, `clock-validation-17` and
  `clock-validation-option-a2`. Before any host change it refuses unless
  exactly one process holds the master of the emulator's pty (the
  `/dev/ptmx` descriptor whose fdinfo names that pty) and every one of its
  threads has an affinity without CPU 3 (the dry run checks this too). It
  records the emulator's pid, start time, threads and every thread's
  affinity before the first step (`emulator_start=`), and checks them again
  after each session (`emulator_A=`, `emulator_B=`): another process holding
  the pty, or a thread that may run on CPU 3, stops the experiment there, as
  any failed step does.
- `tools/emulator_residency.py <timer run>.csv --experiment
  <experiment.record>` reads the recorded emulator and its threads (and any
  thread it is seen to create) and checks the timer capture inside the
  probe's window: no switch to an emulator thread on CPU 3, no wakeup of one
  onto CPU 3 and no migration of one onto CPU 3, and the emulator seen to
  run. On attempt 15's capture, with its emulator allowed on every CPU, it
  finds 139 runs, 42 wakeups and 57 migrations there, and fails.

The operator, as before, on the emulator only, with the arm powered off:

0. Stop any running emulator (Ctrl-C), then launch a new one with CPU 3
   excluded from the start, and give its servos their return delay:

    ```sh
    taskset -c 0-2,4-31 ./build/apps/dxl_emu --link /tmp/rtctrl-rehearsal-tty --firmware 47
    for id in 2 3 4 5 6 7 8 9; do ./build/apps/dxl_inspect --port /tmp/rtctrl-rehearsal-tty write $id return_delay_time 0; done
    for id in 2 3 4 5 6 7 8 9; do ./build/apps/dxl_inspect --port /tmp/rtctrl-rehearsal-tty read $id return_delay_time; done
    grep Cpus_allowed_list /proc/$(pgrep -x dxl_emu)/task/*/status
    ```

   (the emulator in its own terminal; the last line must show `0-2,4-31` for
   every thread); then `sudo prlimit --pid $$ --rtprio=90:90` in the shell
   that runs the supervisor, inside `tmux`.
1. `tools/option_a.sh --dry-run`, then `echo "exit $?"`: exit 0, with the
   emulator's recorded state printed, and nothing changed.
2. `tools/option_a.sh`, then `echo "exit $?"`: the sessions' commands are the
   first run's with `--attempt 16 --outdir ~/rtctrl-hw/clock-validation-16`
   and `--attempt 17 --outdir ~/rtctrl-hw/clock-validation-17`.

The validation then passes only if, besides every criterion below, the
experiment record holds the three emulator states (the same pid and start
time, every thread without CPU 3) and `tools/emulator_residency.py` passes on
the timer capture. Hardware and H2 stay paused.

Nothing is retried, and no budget, guard, rule or load changes. A signal sent
to the supervisor alone lets the running session finish before it stops; a
Ctrl-C or a hangup reaches the session too, which runs its own cleanup first.
The limits: if the supervisor is killed (SIGKILL) or crashes, nothing is
restored automatically: the keeper ends without restoring, and the mask,
the quieted services and both records stay for the operator, as after a
power failure (a reboot resets the mask). After a hangup the sessions' own privileged
cleanup still depends on their sudo, as before.

The validation passes only if all of these hold, judged on the published
captures and records:

- the supervisor exited 0;
- the gate passes on both runs against the closed mask record
  (`--record-state closed`), with the experiment record's session bounds;
- zero read failures and timeouts in both runs;
- the controls passed under their preconditions;
- the timer run's capture is valid, with zero main-trace, instance and
  recorder losses;
- the timekeeping certificate is CERTIFIED with the audited trace-cmd and
  runtime;
- the full-window conversion is accepted: the deadline-slot accounting
  passes and U is at most 5 us;
- no timer wake is unexplained;
- `tools/unbound_work.py` passes on the timer capture;
- every stage is published, verified and released.

The allowed exceptions are the probe criteria T1, T2 and T6a and the
analysis's PTY USB association; nothing else. The untraced run's placement
evidence is the mask's read-back and the kernel's documented policy; the
residency is observed in the timer run. Hardware and H2 stay paused until
the outcome is reviewed and their revised plan accepted, and no failed
historical slot is promoted. This qualifies no control rate.

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
