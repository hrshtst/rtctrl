// dxl_emu's cadence options (apps/bus/dxl_emu_cadence.hpp): each number
// checked before it is scaled (a time's magnitude at most 1e18 ns, a count
// at most 64 bits, nothing infinite or beyond a double), rounded to the
// nanosecond, and then judged by the model's ranges on both sides of each
// bound; each refusal names its reason. The emulator's exit before serving
// is checked on the binary (tests/integration/bus_timing_cadence_smoke.sh).
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "bus/dxl_emu_cadence.hpp"

namespace cad = rtctrl::apps::dxl_emu_cadence;
namespace emu = rtctrl::emu;

namespace {

struct Outcome {
  bool ok = false;
  std::string why;
  cad::CadenceArgs args;
};

Outcome parse(const std::vector<std::string>& args) {
  std::vector<const char*> argv{"dxl_emu"};
  for (const auto& a : args) argv.push_back(a.c_str());
  Outcome o;
  o.ok = cad::parseCadence(static_cast<int>(argv.size()), argv.data(), &o.args, &o.why);
  return o;
}

std::string joined(const std::vector<std::string>& args) {
  std::string s;
  for (const auto& a : args) s += a + " ";
  return s;
}

emu::CadenceConfig accepted(const std::vector<std::string>& args) {
  const Outcome o = parse(args);
  INFO(joined(args) << "-> " << o.why);
  CHECK(o.ok);
  return o.args.config;
}

void refused(const std::vector<std::string>& args, const std::string& why) {
  const Outcome o = parse(args);
  INFO(joined(args) << "-> " << o.why);
  CHECK_FALSE(o.ok);
  CHECK(o.why.find(why) != std::string::npos);
}

const std::vector<std::string> kIndependent{"--cadence", "independent"};

std::vector<std::string> with(std::vector<std::string> base, const std::vector<std::string>& more) {
  base.insert(base.end(), more.begin(), more.end());
  return base;
}

}  // namespace

TEST_CASE("a time is bounded before it is scaled: beyond 1e18 ns, infinite or beyond a double it is out of range",
          "[emu][cadence][dxl_emu]") {
  // The review's case: 1e300 us used to round to INT64_MIN and pass.
  refused(with(kIndependent, {"--cadence-step", "1:1e300"}), "--cadence-step: out of range: 1:1e300");
  refused(with(kIndependent, {"--cadence-step", "1:-1e300"}), "out of range");
  refused(with(kIndependent, {"--cadence-step", "1e300:100"}), "out of range");
  refused(with(kIndependent, {"--cadence-step", "1:1e400"}), "out of range");  // ERANGE, overflow
  refused(with(kIndependent, {"--cadence-step", "1:inf"}), "out of range");
  refused(with(kIndependent, {"--cadence-step", "1:nan"}), "T:S");
  // The bound itself, 1e18 ns, in each unit: at it the parser passes the
  // value to the model; just beyond it, refused before any scaling.
  CHECK(accepted(with(kIndependent, {"--cadence-miss-every", "2", "--cadence-impair-from-s", "1e9"}))
            .impair_from_ns == 1000000000000000000LL);
  refused(with(kIndependent, {"--cadence-miss-every", "2", "--cadence-impair-from-s", "1.000001e9"}),
          "out of range");
  refused(with(kIndependent, {"--cadence-miss-every", "2", "--cadence-impair-from-s", "-1e9"}),
          "impairments' start must be at or after");
  refused(with(kIndependent, {"--cadence-miss-every", "2", "--cadence-impair-from-s", "-1.000001e9"}),
          "out of range");
  refused(with(kIndependent, {"--cadence-phase-us", "1e15"}), "the phase must be within");
  refused(with(kIndependent, {"--cadence-phase-us", "1.000001e15"}), "out of range");
  refused(with(kIndependent, {"--cadence-phase-us", "-1e15"}), "the phase must be within");
  refused(with(kIndependent, {"--cadence-phase-us", "-1.000001e15"}), "out of range");
  refused(with(kIndependent, {"--cadence-byte-ns", "1e18"}), "per byte");
  refused(with(kIndependent, {"--cadence-byte-ns", "1.000001e18"}), "out of range");
  for (const char* flag : {"--cadence-ready-us", "--cadence-delay-us", "--cadence-tie-us"}) {
    refused(with(kIndependent, {flag, "1e16"}), "out of range");
    refused(with(kIndependent, {flag, "-1e16"}), "out of range");
  }
  // Reals: overflow and underflow (ERANGE), infinities, nan.
  refused(with(kIndependent, {"--cadence-ppm", "1e400"}), "out of range");
  refused(with(kIndependent, {"--cadence-ppm", "1e-400"}), "out of range");
  refused(with(kIndependent, {"--cadence-ppm", "-inf"}), "out of range");
  refused(with(kIndependent, {"--cadence-ppm", "nan"}), "not a number");
  refused(with(kIndependent, {"--cadence-ppm", "abc"}), "not a number");
  refused(with(kIndependent, {"--cadence-ppm", "5us"}), "not a number");
  refused(with(kIndependent, {"--cadence-ppm", ""}), "not a number");
  // Finer than a nanosecond, either sign: refused rather than taken as 0.
  refused(with(kIndependent, {"--cadence-phase-us", "0.0004"}), "finer than one nanosecond");
  refused(with(kIndependent, {"--cadence-phase-us", "-0.0004"}), "finer than one nanosecond");
  CHECK(accepted(with(kIndependent, {"--cadence-phase-us", "0.0006"})).phase_ns == 1);
}

TEST_CASE("a time is rounded to the nearest nanosecond, and the model's range is judged on that, on both sides of "
          "each bound",
          "[emu][cadence][dxl_emu]") {
  const auto shift = [](const std::string& s) {
    return accepted(with(kIndependent, {"--cadence-step", "1:" + s})).steps[0].shift_ns;
  };
  CHECK(shift("499.999") == 499999);
  CHECK(shift("-499.999") == -499999);
  CHECK(shift("499.9994") == 499999);
  refused(with(kIndependent, {"--cadence-step", "1:499.9996"}), "below half a period");  // rounds to 500 us
  refused(with(kIndependent, {"--cadence-step", "1:500"}), "below half a period");
  refused(with(kIndependent, {"--cadence-step", "1:-500"}), "below half a period");
  refused(with(kIndependent, {"--cadence-step", "-1:100"}), "at or after the start");
  CHECK(accepted(with(kIndependent, {"--cadence-step", "2.5:-200"})).steps[0].at_ns == 2500000000LL);

  CHECK(accepted(with(kIndependent, {"--cadence-ppm", "1000"})).ppm == 1000.0);
  CHECK(accepted(with(kIndependent, {"--cadence-ppm", "-1000"})).ppm == -1000.0);
  refused(with(kIndependent, {"--cadence-ppm", "1000.001"}), "[-1000, 1000] ppm");
  refused(with(kIndependent, {"--cadence-ppm", "-1000.001"}), "[-1000, 1000] ppm");

  CHECK(accepted(with(kIndependent, {"--cadence-phase-us", "0"})).phase_ns == 0);
  CHECK(accepted(with(kIndependent, {"--cadence-phase-us", "999.999"})).phase_ns == 999999);
  refused(with(kIndependent, {"--cadence-phase-us", "1000"}), "the phase must be within");
  refused(with(kIndependent, {"--cadence-phase-us", "-0.001"}), "the phase must be within");

  CHECK(accepted(with(kIndependent, {"--cadence-ready-us", "1e6"})).ready_ns == 1000000000LL);
  refused(with(kIndependent, {"--cadence-ready-us", "1000000.001"}), "ready time");
  refused(with(kIndependent, {"--cadence-ready-us", "-0.001"}), "ready time");

  CHECK(accepted(with(kIndependent, {"--cadence-byte-ns", "100000"})).byte_ns == 100000);
  refused(with(kIndependent, {"--cadence-byte-ns", "100001"}), "per byte");
  refused(with(kIndependent, {"--cadence-byte-ns", "-1"}), "per byte");

  const std::vector<std::string> every3{"--cadence-delay-every", "3", "--cadence-delay-us"};
  CHECK(accepted(with(kIndependent, with(every3, {"499.999"}))).delay_ns == 499999);
  CHECK(accepted(with(kIndependent, with(every3, {"0.001"}))).delay_ns == 1);
  refused(with(kIndependent, with(every3, {"500"})), "half a period");
  refused(with(kIndependent, with(every3, {"0"})), "half a period");
  refused(with(kIndependent, with(every3, {"-0.001"})), "half a period");

  const std::vector<std::string> tied{"--cadence", "request-tied", "--cadence-tie-us"};
  CHECK(accepted(with(tied, {"30000"})).tie_ns == 30000000);
  CHECK(accepted(with(tied, {"0"})).tie_ns == 0);
  refused(with(tied, {"30000.001"}), "[0, 30000] us");
  refused(with(tied, {"-0.001"}), "[0, 30000] us");
}

TEST_CASE("a count is decimal digits of at most 64 bits", "[emu][cadence][dxl_emu]") {
  const auto miss = [](const std::string& n) { return with(kIndependent, {"--cadence-miss-every", n}); };
  CHECK(accepted(miss("2")).miss_every == 2);
  CHECK(accepted(miss("18446744073709551615")).miss_every == std::numeric_limits<std::uint64_t>::max());
  refused(miss("18446744073709551616"), "--cadence-miss-every: out of range");  // ERANGE
  refused(miss("99999999999999999999999"), "out of range");
  refused(miss("1"), "N >= 2");
  for (const char* bad : {"-1", "-0", "+2", " 2", "2 ", "2x", "0x10", "1e3", ""}) {
    refused(miss(bad), "not a count (decimal digits)");
  }
  refused(with(kIndependent, {"--cadence-delay-every", "18446744073709551616", "--cadence-delay-us", "10"}),
          "--cadence-delay-every: out of range");
  CHECK(accepted(with(kIndependent, {"--cadence-delay-every", "0"})).delay_every == 0);
}

TEST_CASE("the request-tied control refuses the independent cadence's options, the delay among them",
          "[emu][cadence][dxl_emu]") {
  const std::vector<std::string> tied{"--cadence", "request-tied", "--cadence-tie-us", "790"};
  CHECK(accepted(tied).tie_ns == 790000);
  // The review's case: served before, the delay silently ignored.
  refused(with(tied, {"--cadence-delay-us", "120"}), "independent cadence only");
  refused(with(tied, {"--cadence-delay-us", "-120"}), "independent cadence only");
  refused(with(tied, {"--cadence-delay-every", "3"}), "independent cadence only");
  refused(with(tied, {"--cadence-miss-every", "2"}), "independent cadence only");
  refused(with(tied, {"--cadence-ppm", "1"}), "independent cadence only");
  refused(with(tied, {"--cadence-phase-us", "1"}), "independent cadence only");
  refused(with(tied, {"--cadence-step", "1:100"}), "independent cadence only");
  refused(with(tied, {"--cadence-ready-us", "1"}), "independent cadence only");
  refused(with(kIndependent, {"--cadence-tie-us", "150"}), "request-tied cadence only");
}

TEST_CASE("misplaced, unknown and incomplete options; other arguments are left alone",
          "[emu][cadence][dxl_emu]") {
  const Outcome none = parse({"--link", "/tmp/x", "--firmware", "45"});
  CHECK(none.ok);
  CHECK_FALSE(none.args.on);
  refused({"--cadence-ppm", "10"}, "need --cadence");
  refused({"--cadence", "bogus"}, "independent or request-tied");
  refused(with(kIndependent, {"--cadence-ppm"}), "needs a value");
  refused(with(kIndependent, {"--cadence-bogus", "1"}), "--cadence-bogus: unknown option");
  refused(with(kIndependent, {"--cadence-step", "1"}), "T:S");
  refused(with(kIndependent, {"--cadence-step", "1:abc"}), "T:S");
  std::vector<std::string> steps = kIndependent;
  for (int i = 0; i < 17; ++i) {
    steps.push_back("--cadence-step");
    steps.push_back(std::to_string(i) + ":100");
  }
  refused(steps, "at most 16");
}

TEST_CASE("a full set of options gives the configuration it names", "[emu][cadence][dxl_emu]") {
  const Outcome o = parse({"--link", "/tmp/x", "--cadence", "independent", "--cadence-ppm", "-57.5",
                           "--cadence-phase-us", "900", "--cadence-ready-us", "500", "--cadence-byte-ns", "80",
                           "--cadence-delay-every", "10", "--cadence-delay-us", "120", "--cadence-miss-every", "17",
                           "--cadence-impair-from-s", "2.5", "--cadence-step", "3:-200", "--cadence-step", "4.5:300",
                           "--cadence-log", "/tmp/x.csv"});
  INFO(o.why);
  REQUIRE(o.ok);
  CHECK(o.args.on);
  CHECK(o.args.log_path == "/tmp/x.csv");
  const emu::CadenceConfig& c = o.args.config;
  CHECK(c.mode == emu::CadenceMode::kIndependent);
  CHECK(c.ppm == -57.5);
  CHECK(c.phase_ns == 900000);
  CHECK(c.ready_ns == 500000);
  CHECK(c.byte_ns == 80);
  CHECK(c.delay_every == 10);
  CHECK(c.delay_ns == 120000);
  CHECK(c.miss_every == 17);
  CHECK(c.impair_from_ns == 2500000000LL);
  REQUIRE(c.step_count == 2);
  CHECK(c.steps[0].at_ns == 3000000000LL);
  CHECK(c.steps[0].shift_ns == -200000);
  CHECK(c.steps[1].at_ns == 4500000000LL);
  CHECK(c.steps[1].shift_ns == 300000);
}
