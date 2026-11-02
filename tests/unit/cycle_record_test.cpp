#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <thread>
#include <vector>

#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/hw/cycle_record.hpp"
#include "rtctrl/util/spsc_ring.hpp"

namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
using Catch::Approx;
using hw::CommandKind;
using hw::CycleRecord;
using hw::LatencyHistogram;
using rtctrl::util::SpscRing;

namespace {

hw::Config craneConfig() { return hw::Config::load("config/crane_x7.toml"); }

emu::MotorBus busFor(const hw::Config& config) {
  emu::MotorBus bus;
  for (const auto& joint : config.joints) {
    emu::MotorEmulator::Config c;
    c.id = joint.id;
    c.model_number = joint.model_number;
    c.firmware_version = 44;
    bus.add(c);
  }
  return bus;
}

// Stamps that a cycle reached, in the order the cycle reaches them.
std::vector<std::int64_t> orderedStamps(const CycleRecord& r) {
  std::vector<std::int64_t> all = {
      r.wake_ns,          r.read_start_ns,   r.instr_return_ns,
      r.first_reply_ns,   r.last_reply_ns,   r.read_end_ns,
      r.window_open_ns,   r.controller_wake_ns, r.submit_ns,
      r.limit_start_ns,   r.bus_write_start_ns, r.write_return_ns,
      r.cycle_end_ns};
  all.erase(std::remove(all.begin(), all.end(), -1), all.end());
  return all;
}

}  // namespace

TEST_CASE("spsc ring keeps order, refuses when full, and wraps", "[util]") {
  SpscRing<int> ring(3);
  CHECK(ring.capacity() == 4);
  CHECK(SpscRing<int>(0).capacity() == 2);
  int out = 0;
  CHECK_FALSE(ring.pop(&out));
  for (int round = 0; round < 3; ++round) {
    for (int i = 0; i < 4; ++i) CHECK(ring.push(round * 10 + i));
    CHECK_FALSE(ring.push(99));
    for (int i = 0; i < 4; ++i) {
      REQUIRE(ring.pop(&out));
      CHECK(out == round * 10 + i);
    }
    CHECK_FALSE(ring.pop(&out));
  }
}

TEST_CASE("spsc ring hands every value across threads in order", "[util]") {
  constexpr int kCount = 200000;
  SpscRing<int> ring(64);
  std::thread producer([&] {
    for (int i = 0; i < kCount; ++i) {
      while (!ring.push(i)) std::this_thread::yield();
    }
  });
  int expected = 0;
  bool in_order = true;
  while (expected < kCount) {
    int value = 0;
    if (ring.pop(&value)) {
      in_order = in_order && value == expected;
      ++expected;
    } else {
      std::this_thread::yield();
    }
  }
  producer.join();
  CHECK(in_order);
  int extra = 0;
  CHECK_FALSE(ring.pop(&extra));
}

TEST_CASE("latency histogram bins are log spaced with edge bins", "[hw]") {
  CHECK(LatencyHistogram::binOf(0.0) == 0);
  CHECK(LatencyHistogram::binOf(0.5e-6) == 0);
  CHECK(LatencyHistogram::binOf(1e-6) == 1);
  CHECK(LatencyHistogram::binOf(2e-6) == 5);
  CHECK(LatencyHistogram::binOf(1e-3) == 1 + static_cast<int>(std::floor(
                                                 4 * std::log2(1e3))));
  CHECK(LatencyHistogram::binOf(std::exp2(20) * 1e-6) ==
        LatencyHistogram::kBins - 1);
  CHECK(LatencyHistogram::binOf(10.0) == LatencyHistogram::kBins - 1);
  // A value lies below its bin's upper edge, which is a quarter octave
  // above the bin's lower edge.
  for (const double v : {1e-6, 3.3e-5, 7.7e-4, 1e-3, 0.0123, 0.5}) {
    const int b = LatencyHistogram::binOf(v);
    CHECK(v < LatencyHistogram::binUpperEdge(b));
    CHECK(v >= LatencyHistogram::binUpperEdge(b - 1));
  }
}

TEST_CASE("latency histogram quantile edges never undercut the sample",
          "[hw]") {
  LatencyHistogram h;
  CHECK(h.quantileUpperEdge(0.5) == 0.0);
  h.add(-1.0);
  h.add(NAN);
  CHECK(h.count() == 0);

  std::mt19937 rng(7);
  std::lognormal_distribution<double> dist(std::log(4e-4), 0.8);
  std::vector<double> sample;
  for (int i = 0; i < 5000; ++i) {
    sample.push_back(dist(rng));
    h.add(sample.back());
  }
  std::sort(sample.begin(), sample.end());
  CHECK(h.count() == sample.size());
  CHECK(h.max() == sample.back());
  for (const double q : {0.5, 0.9, 0.99, 0.999, 1.0}) {
    const auto rank = static_cast<std::size_t>(
        std::ceil(q * static_cast<double>(sample.size())));
    const double exact = sample[rank - 1];
    const double edge = h.quantileUpperEdge(q);
    CHECK(edge >= exact);
    CHECK(edge <= exact * std::exp2(0.25) * (1 + 1e-12));
  }
}

TEST_CASE("cycle timing accumulates only the intervals a cycle completed",
          "[hw]") {
  CycleRecord r;
  r.scheduled_wake_ns = 1000;
  r.wake_ns = 1500;
  r.read_start_ns = 2000;
  r.read_end_ns = 702000;
  r.window_open_ns = 703000;
  r.limit_start_ns = 900000;
  r.bus_write_start_ns = 910000;
  r.write_return_ns = 950000;
  r.cycle_end_ns = 960000;
  hw::CycleTimingStats stats;
  hw::accumulateCycleTiming(r, -1, &stats);
  CHECK(stats.wake_lateness.count() == 1);
  CHECK(stats.wake_lateness.max() == Approx(500e-9));
  CHECK(stats.period.count() == 0);  // first cycle has no predecessor
  CHECK(stats.host_transaction.max() == Approx(700e-6));
  CHECK(stats.reply_span.count() == 0);
  CHECK(stats.controller_handoff.count() == 0);  // no controller woke
  CHECK(stats.controller_time.count() == 0);
  CHECK(stats.write_handoff.count() == 0);       // not a fresh target
  CHECK(stats.limiter.max() == Approx(10e-6));
  CHECK(stats.host_write.max() == Approx(40e-6));
  CHECK(stats.post_write.max() == Approx(10e-6));
  CHECK(stats.host_work.max() == Approx(958e-6));
  hw::accumulateCycleTiming(r, 500, &stats);
  CHECK(stats.period.count() == 1);
  CHECK(stats.period.max() == Approx(1000e-9));
}

TEST_CASE("timing records separate fresh commands from tagged fallbacks",
          "[hw][timing]") {
  struct Outcome {
    std::vector<CycleRecord> records;
    hw::CraneX7::CycleStats stats;
  };
  const auto exercise = [](std::uint8_t mode) {
    auto config = craneConfig();
    for (auto& joint : config.joints) joint.operating_mode = mode;
    auto bus = busFor(config);
    emu::FakePacketIO io(bus);
    hw::CraneX7::Options options;
    options.control_cycle_s = 0.005;
    options.controller_write_margin_s = 0.001;
    options.timing_ring_capacity = 1024;
    hw::CraneX7 arm(io, config, options);
    REQUIRE(arm.activate());
    REQUIRE(arm.startThread(true));

    // The controller answers every window except one it deliberately
    // skips. A late answer is merely rejected; nothing below depends on
    // how many intended answers made their window.
    std::uint64_t seen = 0;
    for (int k = 0; k < 20; ++k) {
      const auto source = arm.waitFeedbackStamped(seen);
      REQUIRE(source.seq > seen);
      seen = source.seq;
      if (k == 10) continue;
      const std::vector<double> values(config.joints.size(), 0.01 * k);
      if (mode == 3) {
        arm.setTargetPositionsFromFeedback(values, source.seq, source.time);
      } else if (mode == 1) {
        arm.setTargetVelocitiesFromFeedback(values, source.seq,
                                            source.time);
      } else {
        arm.setTargetCurrentsFromFeedback(values, source.seq, source.time);
      }
    }
    arm.stopThread();
    REQUIRE(arm.deactivate());
    Outcome outcome;
    outcome.stats = arm.cycleStats();
    CycleRecord r;
    while (arm.popCycleRecord(&r)) outcome.records.push_back(r);
    return outcome;
  };

  for (const std::uint8_t mode : {std::uint8_t{0}, std::uint8_t{1},
                                  std::uint8_t{3}}) {
    CAPTURE(static_cast<int>(mode));
    const Outcome o = exercise(mode);
    CHECK(o.stats.timing_drops == 0);
    REQUIRE(o.records.size() == o.stats.cycles);

    int fresh = 0;
    int fallbacks_after_fresh = 0;
    // A target accepted as the thread stops is stamped but never written
    // (kind kNone), so controller time counts submissions, not writes.
    std::uint64_t timed_submissions = 0;
    const CycleRecord* last_applied = nullptr;
    for (std::size_t i = 0; i < o.records.size(); ++i) {
      const CycleRecord& r = o.records[i];
      CHECK(r.cycle == i + 1);
      CHECK(r.wake_ns >= r.scheduled_wake_ns);
      if (r.controller_wake_ns >= 0 && r.submit_ns >= 0) ++timed_submissions;
      const auto stamps = orderedStamps(r);
      CHECK(std::is_sorted(stamps.begin(), stamps.end()));
      if (r.command_kind == CommandKind::kFresh) {
        ++fresh;
        // A fresh command carries this cycle's acquisition.
        CHECK(r.read_ok);
        CHECK(r.write_ok);
        CHECK(r.submit_ns >= 0);
        CHECK(r.controller_wake_ns >= 0);
        CHECK(r.applied_source_feedback_seq == r.feedback_seq);
        last_applied = &r;
      } else if (r.command_kind == CommandKind::kFallback ||
                 r.command_kind == CommandKind::kZeroVelocityFallback) {
        CHECK(r.submit_ns == -1);
        if (last_applied != nullptr) {
          ++fallbacks_after_fresh;
          // A retransmission keeps its original target and source tags.
          CHECK(r.applied_target_seq == last_applied->applied_target_seq);
          CHECK(r.applied_source_feedback_seq ==
                last_applied->applied_source_feedback_seq);
          CHECK(r.command_kind == (mode == 1
                                       ? CommandKind::kZeroVelocityFallback
                                       : CommandKind::kFallback));
        }
      }
    }
    CHECK(fresh >= 1);
    // The deliberately skipped window guarantees at least one.
    CHECK(fallbacks_after_fresh >= 1);
    CHECK(o.stats.controller_deadline_misses >= 1);
    CHECK(o.stats.timing.host_transaction.count() == o.stats.cycles);
    CHECK(o.stats.timing.reply_span.count() == o.stats.cycles);
    CHECK(o.stats.timing.controller_time.count() == timed_submissions);
    CHECK(timed_submissions >= static_cast<std::uint64_t>(fresh));
  }
}

TEST_CASE("a full timing ring keeps the oldest records and counts drops",
          "[hw][timing]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.control_cycle_s = 0.002;
  options.timing_ring_capacity = 4;
  hw::CraneX7 arm(io, config, options);
  REQUIRE(arm.activate());
  REQUIRE(arm.startThread());
  std::uint64_t seen = 0;
  while (seen < 12) seen = arm.waitCycle(seen);
  arm.stopThread();
  REQUIRE(arm.deactivate());
  const auto stats = arm.cycleStats();
  std::vector<CycleRecord> records;
  CycleRecord r;
  while (arm.popCycleRecord(&r)) records.push_back(r);
  REQUIRE(records.size() == 4);
  for (std::size_t i = 0; i < records.size(); ++i) {
    CHECK(records[i].cycle == i + 1);
    CHECK(records[i].command_kind == CommandKind::kPeriodic);
  }
  CHECK(stats.timing_drops == stats.cycles - 4);
}

TEST_CASE("timing records are off by default; the statistics are kept",
          "[hw][timing]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.control_cycle_s = 0.002;
  hw::CraneX7 arm(io, config, options);
  REQUIRE(arm.activate());
  REQUIRE(arm.startThread());
  std::uint64_t seen = 0;
  while (seen < 5) seen = arm.waitCycle(seen);
  arm.stopThread();
  REQUIRE(arm.deactivate());
  CycleRecord r;
  CHECK_FALSE(arm.popCycleRecord(&r));
  const auto stats = arm.cycleStats();
  CHECK(stats.timing_drops == 0);
  CHECK(stats.timing.host_transaction.count() == stats.cycles);
  CHECK(stats.timing.host_write.count() == stats.cycles);
  CHECK(stats.timing.period.count() == stats.cycles - 1);
}
