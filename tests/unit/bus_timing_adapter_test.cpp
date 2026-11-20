// x7_bus_timing's adapter timing (apps/bus/bus_timing_adapter.hpp): the
// choice's combination rules, and the worker's driver run cycle by cycle as
// Probe::run runs it (the wake, the read's class from the full-read counter,
// the reply, the close, the publication's own clock read), against synthetic
// bus models with known ground truth: an independent completion lattice, and
// replies tied to the request as the pty emulator's are. The fitter is the
// main thread's FitExchange::service(), at modelled times, or a real thread.
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "bus/bus_timing_adapter.hpp"

namespace ad = rtctrl::hw::adapter;
namespace bt = rtctrl::apps::bus_timing;

namespace {

// The reply's last bytes for a read that began at read_start (kInvalidStamp:
// no complete reply).
using ReplyModel = std::function<std::int64_t(std::int64_t read_start, bool full, std::uint64_t cycle)>;

// An adapter whose completions run on their own lattice, 57.5 ppm fast on the
// host clock (its period 999,942.5 ns): a read's reply is the first
// completion after its bytes are ready (650 us after the read starts for the
// motion window, 700 us for the full one), plus a receive delay of 10 to
// 18 us. At the run's start its completions fall 900 us into each grid
// period, so grid reads meet them until the drift has moved them 240 us
// earlier, after about 4 s.
ReplyModel independentLattice() {
  return [](std::int64_t read_start, bool full, std::uint64_t cycle) {
    const double period = 1e6 * (1 - 57.5e-6);
    const double phase = 900000.0;
    const double ready = static_cast<double>(read_start) + (full ? 700000.0 : 650000.0);
    const double c = phase + std::ceil((ready - phase) / period) * period;
    return static_cast<std::int64_t>(c + 10000.0 + 1000.0 * static_cast<double>((cycle * 7919) % 9));
  };
}

// Replies a constant delay after the request (the pty emulator's kind), with
// a few us of jitter.
ReplyModel requestTied(std::int64_t delay_ns) {
  return [delay_ns](std::int64_t read_start, bool, std::uint64_t cycle) {
    return read_start + delay_ns + 500 * static_cast<std::int64_t>((cycle * 104729) % 7);
  };
}

struct SimRow {
  std::uint64_t cycle = 0;
  std::int64_t scheduled = 0, read_start = 0, stamp = 0, end = 0, publish_at = 0;
  bool full = false;
  std::uint64_t skipped = 0;
  bt::AdapterRow a;
};

struct SimOptions {
  bt::TimingChoice choice;
  std::uint64_t full_every = 100;
  std::int64_t duration_ns = 8000000000LL;
  // The fitter: it fits a queued request this long after the request's close
  // (negative: never), in the order requested.
  std::function<std::int64_t(std::uint64_t generation)> fit_delay = [](std::uint64_t) {
    return std::int64_t{5000000};
  };
  // From a cycle's close to its publication's clock read.
  std::function<std::int64_t(std::uint64_t cycle)> publish_delay = [](std::uint64_t) {
    return std::int64_t{2000};
  };
  int confirm = -1;  // the detector's confirmations, if not as tested
};

struct Sim {
  ad::ScheduleConfig cfg;
  std::unique_ptr<ad::FitExchange> ex;
  std::unique_ptr<bt::AdapterCycles> a;
  std::vector<SimRow> rows;
  std::vector<ad::WakePlan> used;       // used[i]: the plan row i's read executed
  std::vector<ad::WakePlan> published;  // published[i]: the plan row i's close published
  std::uint64_t services = 0;
  bool late_wake_stop = false;
};

bt::TimingChoice adapterChoice(double alternation_us) {
  bt::TimingChoice c;
  c.adapter = true;
  c.rate_hz = 1000.0;
  c.duration_s = 8.0;
  c.bounded = true;
  c.motion_only = true;
  c.full_every = 100;
  c.acquisition_s = 1.0;
  c.settling_s = 0.5;
  c.alternation_us = alternation_us;
  c.alternation_given = alternation_us != 0.0;
  return c;
}

// Probe::run's cycle, with the bus and the clock modelled.
Sim simulate(const ReplyModel& reply, const SimOptions& o) {
  Sim s;
  REQUIRE(bt::timingChoiceError(o.choice).empty());
  s.cfg = bt::scheduleConfig(o.choice);
  if (o.confirm > 0) s.cfg.coupling.confirm = o.confirm;
  s.ex = std::make_unique<ad::FitExchange>(s.cfg.estimator.acquisition_capacity, s.cfg.estimator.fit);
  s.a = std::make_unique<bt::AdapterCycles>(s.cfg, s.ex.get());
  std::deque<std::pair<std::uint64_t, std::int64_t>> requests;  // (generation, requested at)
  std::int64_t wake = s.a->first();
  std::uint64_t full_due_in = 0;
  for (std::uint64_t k = 1;; ++k) {
    const std::int64_t woke = wake + 4000;
    if (woke >= o.duration_ns) {
      s.late_wake_stop = true;
      break;
    }
    // The main thread, concurrently: it fits what is due by now.
    while (!requests.empty()) {
      const std::int64_t d = o.fit_delay(requests.front().first);
      if (d < 0 || woke < requests.front().second + d) break;
      if (s.ex->service()) ++s.services;
      requests.pop_front();
    }
    s.used.push_back(s.a->schedule().current());
    SimRow r;
    r.cycle = k;
    r.scheduled = wake;
    r.read_start = woke + 3000;
    r.full = o.full_every == 1 || (o.full_every > 1 && full_due_in == 0);
    r.stamp = reply(r.read_start, r.full, k);
    const bool read_ok = r.stamp >= 0;
    if (o.full_every > 1) {
      if (!r.full) {
        --full_due_in;
      } else if (read_ok) {
        full_due_in = o.full_every - 1;
      }
    }
    r.end = std::max(read_ok ? r.stamp : r.read_start + 1000000, r.read_start + 100000) + 40000;
    const bool next_full = o.full_every == 1 || (o.full_every > 1 && full_due_in == 0);
    s.a->close(k, read_ok ? r.stamp : ad::kInvalidStamp, r.end, r.end + 1000,
               r.full ? ad::ReadClass::kFull : ad::ReadClass::kMotion,
               next_full ? ad::ReadClass::kFull : ad::ReadClass::kMotion);
    for (std::size_t i = 0; i < s.a->fitCount(); ++i) {
      const bt::FitRecord& f = s.a->fit(i);
      if (f.cycle == k && f.event.outcome == ad::FitOutcome::kRequested) {
        requests.emplace_back(f.event.generation, f.event.at_ns);
      }
    }
    r.publish_at = r.end + 1000 + o.publish_delay(k);
    const ad::WakePlan& p = s.a->publish(r.publish_at, &r.a);
    r.skipped = p.skipped;
    s.published.push_back(p);
    s.rows.push_back(r);
    if (p.wake_ns >= o.duration_ns) break;
    wake = p.wake_ns;
  }
  s.a->finish();
  return s;
}

ad::SkipCounts countRows(const Sim& s, int window /* -1: whole run */) {
  ad::SkipCounts c;
  bool prev = false;
  int prev_w = -2;
  for (const SimRow& r : s.rows) {
    const std::int64_t t = r.scheduled;
    const int w = t < s.cfg.windows.acquisition_ns                                 ? 0
                  : t < s.cfg.windows.acquisition_ns + s.cfg.windows.settling_ns ? 1
                                                                                  : 2;
    const bool over = r.skipped > 0;
    if (window < 0 || w == window) {
      ++c.cycles;
      c.skipped += r.skipped;
      if (over) ++c.overruns;
      if (r.skipped >= 2) ++c.consecutive;
      if (over && prev && (window < 0 || prev_w == w)) ++c.consecutive;
      if (r.a.schedule == 1) ++c.referenced_cycles;
      else ++c.grid_cycles;
    }
    prev = over;
    prev_w = w;
  }
  return c;
}

bool sameCounts(const ad::SkipCounts& x, const ad::SkipCounts& y) {
  return x.cycles == y.cycles && x.skipped == y.skipped && x.overruns == y.overruns &&
         x.consecutive == y.consecutive && x.referenced_cycles == y.referenced_cycles &&
         x.grid_cycles == y.grid_cycles;
}

// Every row's columns against the plans and the accounting.
void checkRecords(const Sim& s) {
  REQUIRE(s.rows.size() == s.used.size());
  REQUIRE(s.rows.size() == s.published.size());
  std::uint64_t bad_plan = 0, bad_close = 0, bad_wake = 0, recheck_over = 0;
  for (std::size_t i = 0; i < s.rows.size(); ++i) {
    const SimRow& r = s.rows[i];
    const ad::WakePlan& u = s.used[i];
    const ad::WakePlan& p = s.published[i];
    // The plan columns are the plan this read executed...
    if (r.scheduled != u.wake_ns || r.a.target_index != u.target_index ||
        r.a.schedule != (u.referenced ? 1 : 0) || r.a.lock_state != static_cast<std::uint8_t>(u.lock_state) ||
        r.a.transition != (u.transition ? 1 : 0) || r.a.predicted_completion_ns != u.predicted_completion_ns ||
        r.a.lead_offset_ns != u.lead_offset_ns || r.a.coupling_state != u.coupling_state) {
      ++bad_plan;
    }
    // ...the published plan of the next row is the one this close made...
    if (i + 1 < s.rows.size() && s.used[i + 1].wake_ns != p.wake_ns) ++bad_plan;
    // ...and the skips are this close's, the recheck's among them.
    if (r.skipped != p.skipped || r.a.recheck_skipped != p.recheck_skipped) ++bad_close;
    if (r.a.recheck_skipped > r.skipped) ++recheck_over;
    if (u.referenced) {
      const std::int64_t want = u.predicted_completion_ns - (s.cfg.planner.lead_ns + u.lead_offset_ns +
                                                             s.cfg.planner.wake_to_read_ns);
      if (std::llabs(u.wake_ns - want) > 1) ++bad_wake;  // one rounding, of the prediction
    } else if (u.wake_ns % s.cfg.planner.grid_period_ns != 0) {
      ++bad_wake;  // the grid is the epoch's whole periods
    }
  }
  CHECK(bad_plan == 0);
  CHECK(bad_close == 0);
  CHECK(recheck_over == 0);
  CHECK(bad_wake == 0);
  // The accounting counts each row once, with the skips its close produced.
  const ad::SkipAccounting& acc = s.a->schedule().accounting();
  CHECK(sameCounts(countRows(s, -1), acc.wholeRun()));
  CHECK(sameCounts(countRows(s, 0), acc.window(ad::Window::kAcquisition)));
  CHECK(sameCounts(countRows(s, 1), acc.window(ad::Window::kSettling)));
  CHECK(sameCounts(countRows(s, 2), acc.window(ad::Window::kLocked)));
  CHECK(s.a->schedule().counts().protocol_errors == 0);
  // No published wake before its own publication's clock read plus the
  // reviewed 20 us margin.
  std::uint64_t early = 0;
  for (std::size_t i = 0; i < s.rows.size(); ++i) {
    if (s.published[i].wake_ns < s.rows[i].publish_at + bt::kPublicationMarginNs) ++early;
  }
  CHECK(early == 0);
}

// The motion-only reads of each lock's lattice alternate +, -, ... from +, over
// the reads executed; full-window reads stay at the base lead; the grid has no
// offset.
void checkOffsets(const Sim& s, std::int64_t delta) {
  std::uint64_t bad = 0, alternations = 0;
  int expect = 1;
  for (std::size_t i = 0; i < s.rows.size(); ++i) {
    const SimRow& r = s.rows[i];
    if (r.a.schedule == 0) {
      if (r.a.lead_offset_ns != 0) ++bad;
      expect = 1;  // a new lock starts at +
      continue;
    }
    if (r.full) {
      if (r.a.lead_offset_ns != 0) ++bad;
      continue;
    }
    if (r.a.lead_offset_ns != expect * delta) ++bad;
    else ++alternations;
    expect = -expect;
  }
  CHECK(bad == 0);
  CHECK(alternations > 0);
}

std::size_t lockCount(const Sim& s) { return s.a->lockCount(); }

}  // namespace

TEST_CASE("the timing choice: grid by default; adapter timing and the alternation only where supported",
          "[bus_timing][adapter]") {
  bt::TimingChoice grid;  // the default
  grid.rate_hz = 500.0;
  grid.duration_s = 1.0;
  CHECK(bt::timingChoiceError(grid).empty());
  CHECK(!grid.adapter);
  const bt::TimingChoice ok = adapterChoice(20.0);
  CHECK(bt::timingChoiceError(ok).empty());
  auto refused = [](bt::TimingChoice c, const char* why) {
    const std::string e = bt::timingChoiceError(c);
    INFO(e);
    CHECK(e.find(why) != std::string::npos);
  };
  {
    bt::TimingChoice c = grid;
    c.alternation_given = true;
    refused(c, "need --timing adapter");
    c = grid;
    c.adapter_values_given = true;
    refused(c, "need --timing adapter");
  }
  {
    bt::TimingChoice c = ok;
    c.rate_hz = 500.0;
    refused(c, "--rate-hz 1000 only");
    c = ok;
    c.calibration = true;
    refused(c, "production traffic only");
    c = ok;
    c.bounded = false;
    refused(c, "--read-path bounded");
    c = ok;
    c.lead_us = 1000.0;
    refused(c, "--lead-us");
    c = ok;
    c.lead_us = std::nan("");
    refused(c, "--lead-us");
    c = ok;
    c.wake_to_read_us = 101.0;
    refused(c, "--wake-to-read-us");
    c = ok;
    c.alternation_us = -1.0;
    refused(c, "--lead-alternation-us must be");
    c = ok;
    c.lead_us = 15.0;  // the -20 us plan would be before its completion by less than nothing
    refused(c, "the lead less the alternation");
    c = ok;
    c.lead_us = 980.0;
    refused(c, "below 1000 us");
    c = ok;
    c.acquisition_s = 4.5;
    refused(c, "--acquisition-s");
    c = ok;
    c.settling_s = -0.1;
    refused(c, "--settling-s");
    c = ok;
    c.duration_s = 1.5;  // acquisition 1 + settling 0.5
    refused(c, "before --duration");
    c = ok;
    c.motion_only = false;
    c.full_every = 1;
    refused(c, "needs motion-only reads");
    c = ok;
    c.full_every = 1;  // an auxiliary interval of at most one period
    refused(c, "needs motion-only reads");
    c = ok;
    c.full_every = 0;  // no full read after the initial one: supported
    CHECK(bt::timingChoiceError(c).empty());
    c = ok;
    c.alternation_us = 0.0;
    c.motion_only = false;
    c.full_every = 1;  // adapter timing without the alternation reads any window
    CHECK(bt::timingChoiceError(c).empty());
  }
  // The configuration: the reviewed margin and freshness, the windows, the lead.
  const ad::ScheduleConfig cfg = bt::scheduleConfig(ok);
  CHECK(cfg.planner.min_gap_ns == 20000);
  CHECK(cfg.fit_max_age_ns == 500000000);
  CHECK(cfg.planner.grid_period_ns == 1000000);
  CHECK(cfg.estimator.nominal_period_ns == 1000000);
  CHECK(cfg.planner.lead_ns == 780000);
  CHECK(cfg.planner.wake_to_read_ns == 10000);
  CHECK(cfg.windows.acquisition_ns == 1000000000);
  CHECK(cfg.windows.settling_ns == 500000000);
  CHECK(cfg.lead_alternation_ns == 20000);
}

TEST_CASE("the schedule observes the reply's last bytes, from a complete read only", "[bus_timing][adapter]") {
  rtctrl::hw::CycleRecord c;
  c.read_ok = true;
  c.first_reply_ns = 1200000;  // the first servo's reply, possibly an earlier completion's
  c.last_reply_ns = 1260000;
  CHECK(bt::replyStamp(c) == 1260000);
  c.read_ok = false;  // a timeout or a malformed reply: no observation
  CHECK(bt::replyStamp(c) == ad::kInvalidStamp);
  c.read_ok = true;
  c.first_reply_ns = c.last_reply_ns = -1;  // no reply stamped
  CHECK(bt::replyStamp(c) == ad::kInvalidStamp);
}

TEST_CASE("each row records the plan its read executed, apart from the skips its close produced",
          "[bus_timing][adapter]") {
  // An independent lattice with the alternation, every 37th publication's
  // clock read 2.5 periods after its close (a scheduler delay).
  SimOptions o;
  o.choice = adapterChoice(20.0);
  o.publish_delay = [](std::uint64_t k) { return k % 37 == 0 ? std::int64_t{2500000} : std::int64_t{2000}; };
  const Sim s = simulate(independentLattice(), o);
  checkRecords(s);
  checkOffsets(s, 20000);
  REQUIRE(lockCount(s) == 1);
  const bt::LockRecord& l = s.a->lock(0);
  CHECK(l.ended_cycle == 0);  // held to the run's end
  CHECK(l.referenced_cycles > 5000);
  CHECK(l.first_declaration == 1);  // no coupling detected
  CHECK(l.first_declaration_after < 150);
  CHECK(l.coupling_at_end == 1);
  // The rechecks moved plans, and their slots are in the rows' skips, once.
  std::uint64_t rechecked = 0, recheck_slots = 0;
  for (const SimRow& r : s.rows) {
    if (r.a.recheck_skipped > 0) ++rechecked;
    recheck_slots += r.a.recheck_skipped;
  }
  CHECK(rechecked > 100);
  // A delayed row's own plan columns are unchanged by its recheck: the next
  // row executes the moved plan.
  std::uint64_t moved_ok = 0;
  for (std::size_t i = 0; i + 1 < s.rows.size(); ++i) {
    if (s.rows[i].a.recheck_skipped == 0 || s.used[i + 1].referenced == false) continue;
    if (s.rows[i + 1].a.target_index == s.published[i].target_index &&
        s.rows[i + 1].a.lead_offset_ns == s.published[i].lead_offset_ns) {
      ++moved_ok;
    }
  }
  CHECK(moved_ok > 50);
  CHECK(recheck_slots <= s.a->schedule().accounting().wholeRun().skipped);
  // The observations: stored then gated (or late) once locked.
  std::uint64_t stored = 0, gated = 0;
  for (const SimRow& r : s.rows) {
    if (r.a.observation == static_cast<std::uint8_t>(ad::Observation::kStored)) ++stored;
    if (r.a.observation == static_cast<std::uint8_t>(ad::Observation::kGated)) ++gated;
  }
  CHECK(stored > 900);
  CHECK(gated > 5000);
  // One request, its result applied, with its age and latency on record.
  REQUIRE(s.a->fitCount() == 2);
  CHECK(s.a->fit(0).event.outcome == ad::FitOutcome::kRequested);
  CHECK(s.a->fit(1).event.outcome == ad::FitOutcome::kApplied);
  CHECK(s.a->fit(1).event.generation == s.a->fit(0).event.generation);
  const std::int64_t age = s.a->fit(1).event.at_ns - s.a->fit(1).event.window_end_ns;
  CHECK(age >= 5000000);   // the fitter's delay
  CHECK(age < 10000000);
  CHECK(s.a->fit(1).event.fit.share > 0.99);
}

TEST_CASE("a false lock on replies tied to the request: early replies, a loss, the fallback and reacquisition, "
          "with the coupling check left without evidence",
          "[bus_timing][adapter][safety]") {
  // A reply 150 us after the request, as a fast pty answers: on the 1 ms grid
  // the stamps form a lattice and the fit locks; on that lattice each request
  // goes out 790 us before its predicted completion, and so does its reply.
  SimOptions o;
  o.choice = adapterChoice(20.0);
  const Sim s = simulate(requestTied(150000), o);
  checkRecords(s);
  checkOffsets(s, 20000);
  const ad::EstimatorCounts& e = s.a->schedule().estimator().counts();
  REQUIRE(lockCount(s) >= 4);
  CHECK(e.losses >= lockCount(s) - 1);
  CHECK(e.lost_no_gated == e.losses);
  std::uint64_t bad = 0;
  for (std::size_t i = 0; i < lockCount(s); ++i) {
    const bt::LockRecord& l = s.a->lock(i);
    if (l.ended_cycle == 0) continue;  // the last may be held at the run's end
    if (l.cause != ad::LossCause::kNoGatedStamp || l.referenced_cycles != 64 || l.coupling_at_end != 0 ||
        l.first_declaration != 0) {
      ++bad;
    }
  }
  CHECK(bad == 0);
  // Each loss's next plan is the grid's, marked as a transition.
  std::uint64_t fallbacks = 0, early = 0;
  for (std::size_t i = 1; i < s.rows.size(); ++i) {
    if (s.rows[i - 1].a.schedule == 1 && s.rows[i].a.schedule == 0) {
      ++fallbacks;
      CHECK(s.rows[i].a.transition == 1);
    }
    if (s.rows[i].a.schedule == 1 &&
        s.rows[i].a.observation == static_cast<std::uint8_t>(ad::Observation::kEarly)) {
      ++early;
    }
  }
  CHECK(fallbacks == e.losses);
  CHECK(early > 200);
  // 650 us early is off the period's half: every sample rejected, no pair.
  CHECK(s.a->schedule().coupling().rejected() > 200);
  CHECK(s.a->schedule().coupling().evaluations() == 0);
}

TEST_CASE("request-tied replies inside the gate: the lock holds, the alternation detects request-tied timing and "
          "latches it, and the result changes no plan",
          "[bus_timing][adapter][coupling]") {
  // The reply 790 us after the read starts: once locked it lands on the
  // predicted completion and follows the request's +-20 us.
  SimOptions o;
  o.choice = adapterChoice(20.0);
  o.duration_ns = 4000000000LL;
  const Sim s = simulate(requestTied(790000), o);
  checkRecords(s);
  checkOffsets(s, 20000);
  REQUIRE(lockCount(s) >= 1);
  const bt::LockRecord& l = s.a->lock(0);
  CHECK(l.first_declaration == 2);  // request-tied timing detected
  CHECK(l.first_declaration_after < 150);
  CHECK(l.coupling_at_end == 2);  // latched to the lock's end
  // Observational: the same run with a detector that never confirms makes the
  // same plans, skips and locks; only the coupling columns differ.
  SimOptions never = o;
  never.confirm = 1000000;
  const Sim t = simulate(requestTied(790000), never);
  REQUIRE(t.rows.size() == s.rows.size());
  std::uint64_t differ = 0, coupling_differs = 0;
  for (std::size_t i = 0; i < s.rows.size(); ++i) {
    bt::AdapterRow x = s.rows[i].a, y = t.rows[i].a;
    if (x.coupling_state != y.coupling_state) ++coupling_differs;
    x.coupling_state = y.coupling_state = 0;
    if (s.rows[i].scheduled != t.rows[i].scheduled || s.rows[i].skipped != t.rows[i].skipped ||
        x.target_index != y.target_index || x.schedule != y.schedule || x.lock_state != y.lock_state ||
        x.transition != y.transition || x.predicted_completion_ns != y.predicted_completion_ns ||
        x.lead_offset_ns != y.lead_offset_ns || x.observation != y.observation ||
        x.recheck_skipped != y.recheck_skipped) {
      ++differ;
    }
  }
  CHECK(differ == 0);
  CHECK(coupling_differs > 0);
  CHECK(lockCount(t) == lockCount(s));
  CHECK(t.a->lock(0).first_declaration == 0);
}

TEST_CASE("a lock that supervision ends before the evidence is in closes as insufficient evidence",
          "[bus_timing][adapter][coupling]") {
  // 700 us after the read: 100 us early on the lattice, inside the half
  // period (the samples pair) but outside the gate: the lock ends after 64
  // ungated cycles, before 32 pairs and 8 evaluations can declare anything.
  SimOptions o;
  o.choice = adapterChoice(20.0);
  const Sim s = simulate(requestTied(700000), o);
  checkRecords(s);
  REQUIRE(lockCount(s) >= 3);
  std::uint64_t bad = 0;
  for (std::size_t i = 0; i < lockCount(s); ++i) {
    const bt::LockRecord& l = s.a->lock(i);
    if (l.ended_cycle == 0) continue;
    if (l.cause != ad::LossCause::kNoGatedStamp || l.coupling_at_end != 0 || l.first_declaration != 0) ++bad;
  }
  CHECK(bad == 0);
  CHECK(s.a->schedule().coupling().rejected() == 0);  // on-period samples: pairs did form
  CHECK(s.a->schedule().coupling().evaluations() == 0);
}

TEST_CASE("delayed fitting: applied within the age limit; abandoned past it, its late result discarded; with no "
          "fitter the grid runs on",
          "[bus_timing][adapter][safety]") {
  SimOptions o;
  o.choice = adapterChoice(0.0);
  o.duration_ns = 6000000000LL;
  {
    // 300 ms: applied, 300 ms old.
    o.fit_delay = [](std::uint64_t) { return std::int64_t{300000000}; };
    const Sim s = simulate(independentLattice(), o);
    checkRecords(s);
    REQUIRE(s.a->fitCount() == 2);
    const ad::FitEvent& applied = s.a->fit(1).event;
    CHECK(applied.outcome == ad::FitOutcome::kApplied);
    CHECK(applied.at_ns - applied.window_end_ns >= 300000000);
    CHECK(applied.at_ns - applied.window_end_ns < 302000000);
    CHECK(applied.at_ns - s.a->fit(0).event.at_ns >= 300000000);
    CHECK(lockCount(s) == 1);
  }
  {
    // The first fit 700 ms late: abandoned at 500 ms and the acquisition
    // restarted; its result, taken later, discarded as an old generation;
    // the second request, fitted at once, locks.
    o.fit_delay = [](std::uint64_t g) { return g == 1 ? std::int64_t{700000000} : std::int64_t{5000000}; };
    const Sim s = simulate(independentLattice(), o);
    checkRecords(s);
    const ad::ScheduleCounts& k = s.a->schedule().counts();
    CHECK(k.abandoned_requests == 1);
    CHECK(k.old_generation_results == 1);
    CHECK(k.fits_applied == 1);
    std::vector<ad::FitOutcome> outcomes;
    for (std::size_t i = 0; i < s.a->fitCount(); ++i) outcomes.push_back(s.a->fit(i).event.outcome);
    const std::vector<ad::FitOutcome> want = {ad::FitOutcome::kRequested, ad::FitOutcome::kAbandoned,
                                              ad::FitOutcome::kOldGeneration, ad::FitOutcome::kRequested,
                                              ad::FitOutcome::kApplied};
    CHECK(outcomes == want);
    const ad::FitEvent& abandoned = s.a->fit(1).event;
    CHECK(abandoned.at_ns - abandoned.window_end_ns > 500000000);
    CHECK(abandoned.at_ns - abandoned.window_end_ns <= 501100000);  // the first close past the limit
    CHECK(s.a->fit(2).event.generation == 1);
    CHECK(s.a->fit(4).event.generation == 2);
    // The grid ran on throughout the wait: no row was a lattice plan before the lock.
    std::uint64_t referenced_before = 0;
    for (const SimRow& r : s.rows) {
      if (r.end < s.a->lock(0).locked_ns && r.a.schedule == 1) ++referenced_before;
    }
    CHECK(referenced_before == 0);
  }
  {
    // No fitter at all: each request is abandoned; with both buffers queued
    // the next request cannot be made (a queue failure) and the acquisition
    // restarts again; the grid keeps running with its usual skips.
    o.fit_delay = [](std::uint64_t) { return std::int64_t{-1}; };
    const Sim s = simulate(independentLattice(), o);
    checkRecords(s);
    const ad::ScheduleCounts& k = s.a->schedule().counts();
    CHECK(k.fits_requested == 2);
    CHECK(k.abandoned_requests == 2);
    CHECK(k.queue_failures >= 1);
    CHECK(k.fits_applied == 0);
    CHECK(lockCount(s) == 0);
    std::uint64_t referenced = 0;
    for (const SimRow& r : s.rows) referenced += r.a.schedule;
    CHECK(referenced == 0);
    // Every grid slot before the last published wake (the one at or after
    // the deadline that ended the run) was run or skipped, once; the model's
    // drift brings grid misses after about 4 s.
    const std::int64_t last = s.published.back().wake_ns;
    CHECK(last >= o.duration_ns);
    CHECK(s.a->schedule().accounting().wholeRun().skipped > 100);
    CHECK(static_cast<std::int64_t>(s.rows.size() + s.a->schedule().accounting().wholeRun().skipped) ==
          last / 1000000);
  }
}

TEST_CASE("shutdown: a run that ends with its fit pending or its lock held leaves its records complete",
          "[bus_timing][adapter][safety]") {
  SimOptions o;
  o.choice = adapterChoice(20.0);
  o.choice.duration_s = 1.51;
  o.choice.settling_s = 0.5;
  {
    // The deadline 10 ms after the window's end, the fitter 50 ms behind.
    o.duration_ns = 1010000000LL;
    o.choice.duration_s = 1.51;
    o.fit_delay = [](std::uint64_t) { return std::int64_t{50000000}; };
    const Sim s = simulate(independentLattice(), o);
    checkRecords(s);
    CHECK(s.a->schedule().lockState() == ad::LockState::kFitPending);
    REQUIRE(s.a->fitCount() == 1);
    CHECK(s.a->fit(0).event.outcome == ad::FitOutcome::kRequested);
    CHECK(lockCount(s) == 0);
    CHECK(s.rows.back().a.lock_state == static_cast<std::uint8_t>(ad::LockState::kFitPending));
  }
  {
    // The deadline during a lock: its record stays open, with the detector's
    // state at the end.
    o.duration_ns = 2000000000LL;
    o.fit_delay = [](std::uint64_t) { return std::int64_t{5000000}; };
    const Sim s = simulate(independentLattice(), o);
    checkRecords(s);
    REQUIRE(lockCount(s) == 1);
    CHECK(s.a->lock(0).ended_cycle == 0);
    CHECK(s.a->lock(0).coupling_at_end == static_cast<std::uint8_t>(s.a->schedule().coupling().state()));
    CHECK(s.a->lock(0).coupling_at_end == 1);
  }
}

TEST_CASE("the worker and a real fitter thread: hand-off, locks, losses and a stop at any point",
          "[bus_timing][adapter][threads]") {
  // The worker's cycles on this thread with modelled time and no waits; the
  // fitter a thread of its own, as the probe's main thread. Replies tied to the
  // request lock and are lost again and again, so results arrive at every
  // stage; the worker stops wherever it is, then the fitter is stopped and
  // joined, and only then is the exchange destroyed.
  for (int run = 0; run < 3; ++run) {
    bt::TimingChoice c = adapterChoice(20.0);
    c.acquisition_s = 0.6;
    c.settling_s = 0.0;
    c.duration_s = 30.0;
    const ad::ScheduleConfig cfg = bt::scheduleConfig(c);
    auto ex = std::make_unique<ad::FitExchange>(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
    auto a = std::make_unique<bt::AdapterCycles>(cfg, ex.get());
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> served{0};
    std::thread fitter([&] {
      while (!stop.load()) {
        if (ex->service()) served.fetch_add(1);
        else std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    });
    const ReplyModel reply = requestTied(150000);
    std::int64_t wake = a->first();
    std::uint64_t full_due_in = 0;
    const std::uint64_t cycles = 9000 + 1700 * static_cast<std::uint64_t>(run);
    for (std::uint64_t k = 1; k <= cycles; ++k) {
      const std::int64_t read_start = wake + 7000;
      const bool full = full_due_in == 0;
      const std::int64_t stamp = reply(read_start, full, k);
      if (!full) --full_due_in;
      else full_due_in = 99;
      const std::int64_t end = stamp + 40000;
      a->close(k, stamp, end, end + 1000, full ? ad::ReadClass::kFull : ad::ReadClass::kMotion,
               full_due_in == 0 ? ad::ReadClass::kFull : ad::ReadClass::kMotion);
      bt::AdapterRow row;
      wake = a->publish(end + 3000, &row).wake_ns;
      // Let the fitter in: model time runs far faster than real time here, so
      // the worker waits a little while a request is out (enough for most
      // fits to come back within the 500 model ms, even under sanitizers).
      if (a->schedule().lockState() == ad::LockState::kFitPending) {
        std::this_thread::sleep_for(std::chrono::microseconds(300));
      } else if (k % 20 == 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    }
    a->finish();
    stop.store(true);
    fitter.join();
    const ad::ScheduleCounts& k = a->schedule().counts();
    INFO("run " << run << ": requested " << k.fits_requested << ", applied " << k.fits_applied << ", stale "
                << k.stale_results << ", abandoned " << k.abandoned_requests << ", old " << k.old_generation_results
                << ", served " << served.load());
    CHECK(k.protocol_errors == 0);
    // Every request is accounted for exactly once: a result taken (applied,
    // refused or stale), abandoned, or still pending at the stop.
    const std::uint64_t pending = a->schedule().lockState() == ad::LockState::kFitPending ? 1 : 0;
    CHECK(k.fits_requested == k.fits_applied + k.fits_refused + k.stale_results + k.abandoned_requests + pending);
    CHECK(k.old_generation_results <= k.abandoned_requests);
    CHECK(served.load() >= k.fits_applied + k.fits_refused + k.stale_results + k.old_generation_results);
    CHECK(k.fits_applied >= 1);
    a.reset();
    ex.reset();  // after the fitter's join
  }
}
