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
run records every task's CPU, policy and allowed CPUs. The session changes
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
include every device on the controller.

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
  interruption), an instance found is left in place and reported;
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
  tracer failure: missing evidence is never recorded as zero losses.

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
every run record as the session's temporary host changes.

**Conversion and its validation.** The anchors are matched one-to-one, in
order, by their interval pattern, which uses only the TSC frequency from
the sampler, never the conversion's offset; a missing or duplicate anchor
fails the run. A suspect anchor, one whose two stamps disagree (both of its
intervals disagree, by equal and opposite amounts), is left out of the
conversion, its interrupt kept in the timeline, only when the interval
across it agrees, its disagreement is at most 50 us, it lies more than 50
anchors after the previous suspect, and the next 50 intervals agree at the
same offset while no offset of one or two anchors either way agrees with
them all; at most 0.1% of the anchors may be left out, and each is reported
(reviewer decision, 2026-11-07). Equal counts and opposing errors do not by
themselves prove that nothing is missing: the zero loss counts and these
conditions are the evidence, and missing, extra and compensating anchors
fail the regressions. The conversion must cover the whole run. A deadline is converted by linear interpolation between the
anchors around it, and never extrapolated (outside them it is
INDETERMINATE). Interpolation has no residual at its own anchors, so the
uncertainty U is validated independently: the residuals of held-out
anchors (each odd anchor predicted from the even ones) and of the sampler's
pairs, each with half its bracket added, which also detects a systematic
offset of the instance's timestamps (for example a constant skew between
the two callbacks). A U above 5 us fails the run; it is an engineering
acceptance target, not an accuracy claim, and the classification's
boundaries include it.

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
