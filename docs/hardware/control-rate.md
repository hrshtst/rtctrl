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
Read (B3), were measured torque-off on 2026-11-05 (below). Targeted USB and
scheduler diagnostics come next; the rate-derived safety options (Phase C)
are still to come. No rate above 100 Hz is qualified,
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
an interrupted session (Ctrl-C or SIGTERM) included. Before the arm, a rehearsal over `dxl_emu` checks the tracers and the clock
correlation: `--port` the emulator's link (with Return Delay Time 0 written
to the emulated servos) and `--usb-device` the adapter's sysfs directory,
whose capture stays empty while nothing opens its port.

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
