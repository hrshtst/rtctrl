#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/dxl/status_parser.hpp"

namespace dxl = rtctrl::dxl;

namespace {

using Parser = dxl::StatusReplyParser;
using Status = Parser::Status;
using Fault = Parser::Fault;
using Bytes = std::vector<std::uint8_t>;

Bytes hex(const std::string& text) {
  Bytes out;
  std::istringstream in(text);
  unsigned value = 0;
  while (in >> std::hex >> value) {
    out.push_back(static_cast<std::uint8_t>(value));
  }
  return out;
}

Bytes join(std::initializer_list<Bytes> parts) {
  Bytes out;
  for (const Bytes& part : parts) {
    out.insert(out.end(), part.begin(), part.end());
  }
  return out;
}

Bytes packet(std::uint8_t id, std::uint8_t error, const Bytes& data) {
  return dxl::encodeStatusPacket(id, error, data.data(), data.size());
}

Bytes withCrc(Bytes bytes) {
  const std::uint16_t crc = dxl::crc16(bytes.data(), bytes.size());
  bytes.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  bytes.push_back(static_cast<std::uint8_t>(crc >> 8));
  return bytes;
}

}  // namespace

// The example packets of the ROBOTIS Protocol 2.0 e-manual (ping, read and
// sync read), checked against an independent bitwise CRC before use.
TEST_CASE("crc16 matches the ROBOTIS Protocol 2.0 examples", "[dxl][parser]") {
  struct Vector {
    const char* bytes;
    std::uint16_t crc;
  };
  for (const Vector v : {
           Vector{"FF FF FD 00 01 03 00 01", 0x4E19},
           Vector{"FF FF FD 00 01 07 00 55 00 06 04 26", 0x5D65},
           Vector{"FF FF FD 00 01 07 00 02 84 00 04 00", 0x151D},
           Vector{"FF FF FD 00 01 08 00 55 00 A6 00 00 00", 0xC08C},
           Vector{"FF FF FD 00 FE 09 00 82 84 00 04 00 01 02", 0xFACE},
           Vector{"FF FF FD 00 02 08 00 55 00 1F 08 00 00", 0xBEBA},
       }) {
    CAPTURE(v.bytes);
    const Bytes b = hex(v.bytes);
    CHECK(dxl::crc16(b.data(), b.size()) == v.crc);
  }
  // A CRC continues over a further block.
  const Bytes b = hex("FF FF FD 00 01 07 00 55 00 06 04 26");
  CHECK(dxl::crc16(b.data() + 5, b.size() - 5, dxl::crc16(b.data(), 5)) ==
        0x5D65);
}

TEST_CASE("the encoder reproduces the ROBOTIS status examples",
          "[dxl][parser]") {
  const Bytes ping = hex("FF FF FD 00 01 07 00 55 00 06 04 26 65 5D");
  CHECK(packet(1, 0, {0x06, 0x04, 0x26}) == ping);
  const Bytes read1 = hex("FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C0");
  CHECK(packet(1, 0, {0xA6, 0x00, 0x00, 0x00}) == read1);
  const Bytes read2 = hex("FF FF FD 00 02 08 00 55 00 1F 08 00 00 BA BE");
  CHECK(packet(2, 0, {0x1F, 0x08, 0x00, 0x00}) == read2);
}

TEST_CASE("a grouped read's replies parse into data and error bytes",
          "[dxl][parser]") {
  // The e-manual's sync read: Present Position of ids 1 and 2.
  const Bytes wire = hex(
      "FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C0 "
      "FF FF FD 00 02 08 00 55 00 1F 08 00 00 BA BE");
  Parser parser({1, 2}, 4);
  REQUIRE(parser.parse(wire.data(), wire.size()) == Status::kComplete);
  CHECK(parser.fault() == Fault::kNone);
  CHECK(parser.packetsParsed() == 2);
  const Bytes params{0xA6, 0x00, 0x00, 0x00, 0x1F, 0x08, 0x00, 0x00};
  CHECK(parser.params() == params);
  const Bytes errors{0x00, 0x00};
  CHECK(parser.errors() == errors);
  CHECK_FALSE(parser.anyServoError());

  SECTION("every prefix is incomplete, never a fault") {
    for (std::size_t n = 0; n < wire.size(); ++n) {
      CAPTURE(n);
      CHECK(parser.parse(wire.data(), n) == Status::kIncomplete);
    }
    CHECK(parser.parse(wire.data(), 15) == Status::kIncomplete);
    CHECK(parser.packetsParsed() == 1);
  }
  SECTION("parsing again allocates nothing") {
    const std::uint8_t* where = parser.params().data();
    for (int i = 0; i < 3; ++i) {
      REQUIRE(parser.parse(wire.data(), wire.size()) == Status::kComplete);
    }
    CHECK(parser.params().data() == where);
  }
}

TEST_CASE("the production feedback read parses with stuffed negatives",
          "[dxl][parser]") {
  // rtctrl's 13-byte feedback window from ids 2..9: current -1 (FF FF)
  // followed by velocity -3 (FD FF FF FF) forms FF FF FD, which is stuffed.
  const Bytes window{0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0x00,
                     0x08, 0x00, 0x00, 0x78, 0x00, 0x23};
  std::vector<std::uint8_t> ids;
  Bytes wire;
  Bytes expected;
  for (std::uint8_t id = 2; id <= 9; ++id) {
    ids.push_back(id);
    Bytes data = window;
    data[12] = static_cast<std::uint8_t>(30 + id);  // a distinct temperature
    const Bytes p = packet(id, 0, data);
    REQUIRE(p.size() == 11 + 13 + 1);  // one stuffing byte
    wire.insert(wire.end(), p.begin(), p.end());
    expected.insert(expected.end(), data.begin(), data.end());
  }
  Parser parser(ids, 13);
  REQUIRE(parser.parse(wire.data(), wire.size()) == Status::kComplete);
  CHECK(parser.params() == expected);
  CHECK_FALSE(parser.anyServoError());
}

TEST_CASE("byte stuffing is undone, and a missing stuffing byte fails",
          "[dxl][parser]") {
  const Bytes data{0xFF, 0xFF, 0xFD, 0x00};
  const Bytes wire = packet(3, 0, data);
  // FF FF FD gains a stuffing FD, which the length field counts.
  REQUIRE(wire.size() == 11 + 4 + 1);
  CHECK(wire.at(5) == 4 + 4 + 1);
  CHECK(wire.at(11) == 0xFD);
  CHECK(wire.at(12) == 0xFD);
  Parser parser({3}, 4);
  REQUIRE(parser.parse(wire.data(), wire.size()) == Status::kComplete);
  CHECK(parser.params() == data);

  SECTION("a pattern that ends the data still has its stuffing byte") {
    const Bytes tail{0x00, 0xFF, 0xFF, 0xFD};
    const Bytes w = packet(3, 0, tail);
    REQUIRE(parser.parse(w.data(), w.size()) == Status::kComplete);
    CHECK(parser.params() == tail);
  }
  SECTION("without its stuffing byte, the pattern is malformed") {
    // Drop the stuffing FD and correct the length and the CRC: FF FF FD in
    // a body is never followed by anything but FD.
    Bytes w(wire.begin(), wire.end() - 2);
    w.erase(w.begin() + 12);
    w[5] = 4 + 4;
    w = withCrc(w);
    CHECK(parser.parse(w.data(), w.size()) == Status::kMalformed);
    CHECK(parser.fault() == Fault::kStuffing);
    CHECK(parser.faultOffset() == 12);
  }
  SECTION("a stuffing byte that leaves the data one byte short") {
    // A correct CRC over a body whose stuffing byte takes the place of the
    // last data byte: three data bytes for a request of four.
    const Bytes w = withCrc(hex("FF FF FD 00 03 08 00 55 00 FF FF FD FD"));
    CHECK(parser.parse(w.data(), w.size()) == Status::kMalformed);
    CHECK(parser.fault() == Fault::kParamCount);
    CHECK(parser.faultOffset() == 13);
  }
  SECTION("a pattern at the end of the body without its stuffing byte") {
    Bytes w = hex("FF FF FD 00 03 08 00 55 00 00 FF FF FD");
    w = withCrc(w);
    CHECK(parser.parse(w.data(), w.size()) == Status::kMalformed);
    CHECK(parser.fault() == Fault::kStuffing);
    CHECK(parser.faultOffset() == 13);
  }
}

TEST_CASE("each fault is found at the byte that rules out a valid reply",
          "[dxl][parser]") {
  const Bytes first = packet(2, 0, {1, 2, 3, 4});
  const Bytes second = packet(3, 0, {5, 6, 7, 8});
  const Bytes wire = join({first, second});
  Parser parser({2, 3}, 4);
  REQUIRE(parser.parse(wire.data(), wire.size()) == Status::kComplete);
  const auto expectFault = [&parser](const Bytes& bytes, Fault fault,
                                     std::size_t offset) {
    CHECK(parser.parse(bytes.data(), bytes.size()) == Status::kMalformed);
    CHECK(parser.fault() == fault);
    CHECK(parser.faultOffset() == offset);
  };

  SECTION("garbage before the header") {
    expectFault(join({{0x00}, wire}), Fault::kHeader, 0);
  }
  SECTION("a reserved byte that is not zero") {
    Bytes w = wire;
    w[3] = 0x01;
    expectFault(w, Fault::kReserved, 3);
  }
  SECTION("a reply from another servo, as soon as its id arrives") {
    const Bytes other = packet(4, 0, {1, 2, 3, 4});
    expectFault(Bytes(other.begin(), other.begin() + 5), Fault::kId, 4);
  }
  SECTION("the replies out of order") {
    expectFault(join({second, first}), Fault::kId, 4);
  }
  SECTION("a late reply of the normal length during recovery") {
    // The recovery read asks for one byte more, so a late reply to the
    // normal request fails on its length field.
    Parser recovery({2, 3}, 5);
    CHECK(recovery.parse(wire.data(), wire.size()) == Status::kMalformed);
    CHECK(recovery.fault() == Fault::kLength);
    CHECK(recovery.faultOffset() == 5);
  }
  SECTION("a length beyond any stuffing") {
    Bytes w = wire;
    w[5] = 0xFF;
    expectFault(w, Fault::kLength, 5);
  }
  SECTION("not a status packet") {
    Bytes w = wire;
    w[7] = 0x02;
    expectFault(w, Fault::kInstruction, 7);
  }
  SECTION("a corrupted data byte fails the CRC") {
    Bytes w = wire;
    w[10] ^= 0x01;
    expectFault(w, Fault::kCrc, 13);
  }
  SECTION("a corrupted second reply") {
    Bytes w = wire;
    w[first.size() + 9] ^= 0x80;
    expectFault(w, Fault::kCrc, first.size() + 13);
  }
  SECTION("a byte after the last reply") {
    expectFault(join({wire, {0x00}}), Fault::kExtraBytes, wire.size());
  }
  SECTION("more data than the request") {
    // The length claims one stuffing byte, but the body carries a fifth
    // data byte instead.
    const Bytes w = withCrc(hex("FF FF FD 00 02 09 00 55 00 01 02 03 04 05"));
    expectFault(w, Fault::kParamCount, 13);
  }
  SECTION("a truncated reply stays incomplete until the deadline") {
    CHECK(parser.parse(wire.data(), wire.size() - 1) == Status::kIncomplete);
    CHECK(parser.fault() == Fault::kNone);
  }
}

TEST_CASE("a servo's error byte fails the read without making it malformed",
          "[dxl][parser]") {
  // Id 3 sets the Alert bit (a hardware error is pending).
  const Bytes wire = join({packet(2, 0x00, {1, 2, 3, 4}),
                           packet(3, 0x80, {5, 6, 7, 8})});
  Parser parser({2, 3}, 4);
  REQUIRE(parser.parse(wire.data(), wire.size()) == Status::kComplete);
  CHECK(parser.anyServoError());
  const Bytes errors{0x00, 0x80};
  CHECK(parser.errors() == errors);
  const Bytes params{1, 2, 3, 4, 5, 6, 7, 8};
  CHECK(parser.params() == params);
}

TEST_CASE("no single corrupted byte passes as a complete reply",
          "[dxl][parser]") {
  const Bytes wire = join({packet(2, 0, {0x10, 0x20, 0x30, 0x40}),
                           packet(3, 0, {0xFF, 0xFF, 0xFD, 0x00})});
  Parser parser({2, 3}, 4);
  REQUIRE(parser.parse(wire.data(), wire.size()) == Status::kComplete);
  int passed = 0;
  for (std::size_t i = 0; i < wire.size(); ++i) {
    for (int x = 1; x < 256; ++x) {
      Bytes w = wire;
      w[i] = static_cast<std::uint8_t>(w[i] ^ x);
      if (parser.parse(w.data(), w.size()) == Status::kComplete) ++passed;
    }
  }
  CHECK(passed == 0);
}

TEST_CASE("the parser refuses a request no servo can answer",
          "[dxl][parser]") {
  CHECK_THROWS_AS(Parser({}, 4), std::invalid_argument);
  CHECK_THROWS_AS(Parser({2}, 0), std::invalid_argument);
  CHECK_THROWS_AS(Parser({253}, 4), std::invalid_argument);
  CHECK_THROWS_AS(Parser({2, 2}, 4), std::invalid_argument);
  const Parser parser({2, 3}, 13);
  CHECK(parser.minLengthField() == 17);
  CHECK(parser.maxLengthField() == 21);
}
