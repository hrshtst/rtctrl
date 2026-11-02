# Control-rate qualification

rtctrl's hardware loops run at 100 Hz, the only rate validated on the arm.
This page is the procedure for measuring and qualifying higher rates (200, 500
and 1000 Hz), one operating mode at a time. It follows the approved plan
(revision 4, 2026-11-02). The 100 Hz default does not change here, and nothing
on this page changes a controller gain or the `x7_track` scale cap.

**Status.** The measurement tooling of Phase A exists: the torque-off bus
probe, the per-cycle timing log, the posture references and the timing
summary. The exploratory steps E0 and E1 below can run now. The bounded bus
transaction, the communication options (Return Delay Time 0, Fast Sync Read,
a motion-only read) and the rate-derived safety options come after
checkpoint 1, so no rate above 100 Hz is qualified yet. Every rate other than
100 Hz prints an experimental warning.

## What the numbers mean

Every timing quantity belongs to one of three classes, and reports keep them
apart:

- **Measured**: host-clock intervals between stamps the host took (wake
  lateness, the host transaction interval, the reply span, controller time,
  limiter and host write times, host work per cycle). These are exact for the
  recorded cycles.
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
  alongside, or, if stress-ng is unavailable, a parallel clean rebuild of
  rtctrl. Record the exact command.
- Go 200, then 500, then 1000 Hz per mode, and attempt a rate only after
  every criterion passed at the previous one.
- Never position the arm at a rate that has not qualified. Powered steps at a
  new rate start from the measured posture (`--hold-current`,
  `--sweep-current`), which needs no homing.
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
probe's goal writes would switch torque on. The inventory prints a warning for
such a servo; report it at checkpoint 1 before changing any servo setting.

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

`--confirm-writes` reads back the last servo's goal after every write,
alternating a harmless one-unit offset (torque is off) so each write is
distinguishable. The acknowledging read cannot be sent before the write, and
its reply cannot arrive before its own serialization and the servo's Return
Delay Time. That gives each write an upper bound on its wire completion,
`C+ = ack_return - ack_wire - return_delay`. `--drain` stamps `tcdrain()`,
whose meaning is adapter-specific; it is reported beside `C+`.

Each run writes a per-cycle CSV and a summary TOML (`<log>.summary.toml`) with
the run kind, the analytic wire budget, failure counts, the achieved frequency
and the phase histograms (bin upper edges, never below the recorded quantile).

### Summaries

```sh
uv run --project tools tools/timing_summary.py rate-e1-prod-current-500hz-<date>-1.csv \
  --rate-hz 500 --check
```

`timing_summary.py` reads probe logs and app `--timing-log` files, prints
every quantity with its class, and with `--check` evaluates the criteria
below. Calibration logs also print the completion-offset distribution and an
envelope candidate (1.25 times its maximum). `x7_controller_bench` measures
the controller's own computation off-hardware for comparison.

### Checkpoint 1

The reviewer receives the E0 inventory and the E1 summaries. From them they
select the communication changes (Phase B), set provisional envelopes, and
decide whether current-based position at 1 kHz is wanted (it needs 4 Mbps at
the 70% headroom rule).

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
