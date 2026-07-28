// Shared app-plumbing regressions: the verified-shutdown guard (a
// hardware app must never print success over an unclean or throwing
// deactivation) and the strict, order-independent CLI parse (an
// unknown flag falling through atof once ran the arm at minimum
// scale; a lone trailing --port was silently ignored).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <string>
#include <vector>

#include "x7_common.hpp"

namespace {

// parseCli takes char*[] — build mutable argv storage.
struct Argv {
  explicit Argv(std::initializer_list<const char*> args) {
    store.emplace_back("app");
    for (const char* a : args) store.emplace_back(a);
    for (auto& s : store) ptrs.push_back(s.data());
  }
  int argc() const { return static_cast<int>(ptrs.size()); }
  char** argv() { return ptrs.data(); }
  std::vector<std::string> store;
  std::vector<char*> ptrs;
};

struct FakeHw {
  int deactivates = 0;
  int quiesces = 0;
  bool ok = true;
  bool throws = false;
  bool deactivate() {
    ++deactivates;
    if (throws) throw std::runtime_error("bus died mid-shutdown");
    return ok;
  }
  void requestQuiesce() { ++quiesces; }
};

}  // namespace

TEST_CASE("shutdown guard: clean deactivation, idempotent run",
          "[x7_common]") {
  FakeHw hw;
  {
    x7::ShutdownGuardT<FakeHw> guard{hw};
    CHECK(guard.run());
    CHECK(hw.deactivates == 1);
    CHECK(hw.quiesces == 0);
    // second run() must not deactivate again
    CHECK(guard.run());
    CHECK(hw.deactivates == 1);
  }
  // destructor after an explicit run() adds nothing
  CHECK(hw.deactivates == 1);
}

TEST_CASE("shutdown guard: failed deactivation quiesces exactly once",
          "[x7_common][safety]") {
  FakeHw hw;
  hw.ok = false;
  x7::ShutdownGuardT<FakeHw> guard{hw};
  CHECK_FALSE(guard.run());
  CHECK(hw.deactivates == 1);
  CHECK(hw.quiesces == 1);
  CHECK_FALSE(guard.run());  // still reports unclean, no re-run
  CHECK(hw.deactivates == 1);
  CHECK(hw.quiesces == 1);
}

TEST_CASE("shutdown guard: throwing deactivation is an unclean "
          "shutdown, nothing propagates",
          "[x7_common][safety]") {
  // deactivate() is not noexcept: a throw must still silence the bus
  // and report — and must not escape run() (review finding)
  FakeHw hw;
  hw.throws = true;
  x7::ShutdownGuardT<FakeHw> guard{hw};
  bool clean = true;
  REQUIRE_NOTHROW(clean = guard.run());
  CHECK_FALSE(clean);
  CHECK(hw.deactivates == 1);
  CHECK(hw.quiesces == 1);
}

TEST_CASE("shutdown guard: destructor is the exception net",
          "[x7_common][safety]") {
  SECTION("scope exit without an explicit run() deactivates") {
    FakeHw hw;
    { x7::ShutdownGuardT<FakeHw> guard{hw}; }
    CHECK(hw.deactivates == 1);
    CHECK(hw.quiesces == 0);
  }
  SECTION("failing variant still quiesces from the destructor") {
    FakeHw hw;
    hw.ok = false;
    { x7::ShutdownGuardT<FakeHw> guard{hw}; }
    CHECK(hw.deactivates == 1);
    CHECK(hw.quiesces == 1);
  }
  SECTION("throwing variant neither propagates nor skips the quiesce") {
    FakeHw hw;
    hw.throws = true;
    REQUIRE_NOTHROW([&] { x7::ShutdownGuardT<FakeHw> guard{hw}; }());
    CHECK(hw.deactivates == 1);
    CHECK(hw.quiesces == 1);
  }
}

TEST_CASE("parseCli: front flags keep working", "[x7_common]") {
  Argv a{"--config", "c.toml", "--port", "/dev/x", "0.5"};
  const auto cli = x7::parseCli(a.argc(), a.argv());
  CHECK(cli.ok);
  CHECK(cli.config_path == "c.toml");
  CHECK(cli.port_override == "/dev/x");
  REQUIRE(cli.rest.size() == 1);
  CHECK(std::string(cli.rest[0]) == "0.5");
}

TEST_CASE("parseCli: flags after positionals are consumed "
          "(order-independence)",
          "[x7_common]") {
  // the old front-only parse fed "--port" through atof into the scale
  Argv a{"3.5", "--port", "/dev/x"};
  const auto cli = x7::parseCli(a.argc(), a.argv());
  CHECK(cli.ok);
  CHECK(cli.port_override == "/dev/x");
  REQUIRE(cli.rest.size() == 1);
  CHECK(std::string(cli.rest[0]) == "3.5");
}

TEST_CASE("parseCli: interleaved flags preserve app-token order",
          "[x7_common]") {
  Argv a{"--kp", "5", "--config", "c.toml", "0.4"};
  const auto cli = x7::parseCli(a.argc(), a.argv());
  CHECK(cli.ok);
  CHECK(cli.config_path == "c.toml");
  REQUIRE(cli.rest.size() == 3);
  CHECK(std::string(cli.rest[0]) == "--kp");
  CHECK(std::string(cli.rest[1]) == "5");
  CHECK(std::string(cli.rest[2]) == "0.4");
}

TEST_CASE("parseCli: a flag missing its value is an error, not a "
          "silent positional",
          "[x7_common]") {
  SECTION("lone trailing --port") {
    Argv a{"5", "--port"};
    CHECK_FALSE(x7::parseCli(a.argc(), a.argv()).ok);
  }
  SECTION("lone trailing --config") {
    Argv a{"--config"};
    CHECK_FALSE(x7::parseCli(a.argc(), a.argv()).ok);
  }
}

TEST_CASE("parseCli: duplicates last-wins; bare argv is defaults",
          "[x7_common]") {
  SECTION("duplicate --port") {
    Argv a{"--port", "/dev/a", "--port", "/dev/b"};
    const auto cli = x7::parseCli(a.argc(), a.argv());
    CHECK(cli.ok);
    CHECK(cli.port_override == "/dev/b");
  }
  SECTION("no arguments") {
    Argv a{};
    const auto cli = x7::parseCli(a.argc(), a.argv());
    CHECK(cli.ok);
    CHECK(cli.config_path == "config/crane_x7.toml");
    CHECK(cli.port_override.empty());
    CHECK(cli.rest.empty());
  }
  SECTION("documented limitation: a token equal to a flag name is "
          "always the flag") {
    Argv a{"--config", "--port"};
    const auto cli = x7::parseCli(a.argc(), a.argv());
    CHECK(cli.ok);
    CHECK(cli.config_path == "--port");
    CHECK(cli.rest.empty());
  }
}

TEST_CASE("strict numeric parsing", "[x7_common]") {
  double d = -1.0;
  CHECK(x7::parseStrictDouble("1.5", &d));
  CHECK(d == Catch::Approx(1.5));
  CHECK(x7::parseStrictDouble("-0.3", &d));
  CHECK_FALSE(x7::parseStrictDouble("garbage", &d));
  CHECK_FALSE(x7::parseStrictDouble("1.5x", &d));
  CHECK_FALSE(x7::parseStrictDouble("", &d));
  CHECK_FALSE(x7::parseStrictDouble("nan", &d));
  CHECK_FALSE(x7::parseStrictDouble("inf", &d));

  long l = -1;
  CHECK(x7::parseStrictLong("6", &l));
  CHECK(l == 6);
  CHECK(x7::parseStrictLong("-2", &l));
  CHECK(l == -2);
  CHECK_FALSE(x7::parseStrictLong("6.5", &l));
  CHECK_FALSE(x7::parseStrictLong("x", &l));
  CHECK_FALSE(x7::parseStrictLong("", &l));
  // ERANGE overflow must reject, never wrap (review finding)
  CHECK_FALSE(x7::parseStrictLong("99999999999999999999999999", &l));
}
