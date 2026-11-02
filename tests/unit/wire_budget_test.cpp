#include "rtctrl/dxl/wire_budget.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using Catch::Approx;
namespace wire = rtctrl::dxl::wire;
using wire::ReadVariant;

namespace {
constexpr int kServos = 8;
constexpr int kBaud3M = 3000000;
constexpr int kBaud4M = 4000000;

double millis(int bytes, int baud) {
  return bytes * wire::byteTimeS(baud) * 1e3;
}
}  // namespace

TEST_CASE("sync read and write sizes match the protocol framing", "[dxl]") {
  // 10 framing bytes + start address + length + one byte per id.
  CHECK(wire::syncReadInstructionBytes(kServos) == 22);
  CHECK(wire::syncReadReplyBytes(kServos, 13) == 8 * 24);
  CHECK(wire::syncReadStuffingMaxBytes(kServos, 13) == 8 * 4);
  CHECK(wire::fastSyncReadReplyBytes(kServos, 13) == 8 + 8 * 17);
  CHECK(wire::syncWriteBytes(kServos, 2) == 38);
  CHECK(wire::syncWriteBytes(kServos, 4) == 54);
  CHECK(wire::syncWriteBytes(kServos, 10) == 102);
  CHECK(wire::readInstructionBytes() == 14);
  CHECK(wire::readReplyBytes(4) == 15);
}

TEST_CASE("today's cycle traffic matches the reviewed budget", "[dxl]") {
  // The reviewer's minimum serial times at 3 Mbps (no stuffing) and the
  // plan's worst-case table (stuffing included).
  const auto current = wire::cycleBytes(kServos, ReadVariant::kSync, 13,
                                        wire::kWriteWidthCurrent);
  const auto position = wire::cycleBytes(kServos, ReadVariant::kSync, 13,
                                         wire::kWriteWidthPosition);
  const auto cbp = wire::cycleBytes(kServos, ReadVariant::kSync, 13,
                                    wire::kWriteWidthCurrentBasedPosition);
  CHECK(current.total(false) == 252);
  CHECK(position.total(false) == 268);
  CHECK(cbp.total(false) == 316);
  CHECK(millis(current.total(false), kBaud3M) == Approx(0.840));
  CHECK(millis(position.total(false), kBaud3M) == Approx(0.89333));
  CHECK(millis(cbp.total(false), kBaud3M) == Approx(1.05333));
  CHECK(current.total(true) == 284);
  CHECK(position.total(true) == 300);
  CHECK(cbp.total(true) == 348);
  CHECK(millis(current.total(true), kBaud3M) == Approx(0.94667));
  CHECK(millis(cbp.total(true), kBaud4M) == Approx(0.870));
}

TEST_CASE("fast sync read variants match the plan's budget table", "[dxl]") {
  const auto fast = [](int len, int width) {
    return wire::cycleBytes(kServos, ReadVariant::kFastSync, len, width)
        .total(true);
  };
  CHECK(fast(10, 2) == 180);
  CHECK(fast(13, 2) == 204);
  CHECK(fast(10, 4) == 196);
  CHECK(fast(13, 4) == 220);
  CHECK(fast(10, 6) == 212);
  CHECK(fast(13, 6) == 236);
  CHECK(millis(fast(10, 6), kBaud3M) == Approx(0.70667));
  CHECK(millis(fast(13, 6), kBaud3M) == Approx(0.78667));
  CHECK(millis(fast(13, 4), kBaud3M) == Approx(0.73333));
  CHECK(millis(fast(10, 2), kBaud4M) == Approx(0.450));
  // Fast Sync Read replies are never stuffed.
  CHECK(wire::cycleBytes(kServos, ReadVariant::kFastSync, 13, 2)
            .read_stuffing_max == 0);
}
