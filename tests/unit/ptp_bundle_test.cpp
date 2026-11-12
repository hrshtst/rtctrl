#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "plan/ptp_bundle.hpp"

namespace {

namespace fs = std::filesystem;

fs::path scratchDir() {
  const fs::path dir = "build/ptp_bundle_test";
  fs::create_directories(dir);
  return dir;
}

void writeBytes(const fs::path& path, const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output << bytes;
  output.close();
  REQUIRE(output);
}

}  // namespace

TEST_CASE("sha256File gives the standard SHA-256 vectors", "[plan][bundle]") {
  const fs::path path = scratchDir() / "vector.bin";
  fs::remove(path);

  SECTION("empty file") {
    writeBytes(path, "");
    CHECK(x7::ptp::sha256File(path) ==
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  }
  SECTION("abc") {
    writeBytes(path, "abc");
    REQUIRE(fs::file_size(path) == 3);
    CHECK(x7::ptp::sha256File(path) ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  }
  SECTION("one million a") {
    writeBytes(path, std::string(1000000, 'a'));
    REQUIRE(fs::file_size(path) == 1000000);
    CHECK(x7::ptp::sha256File(path) ==
          "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  }
  fs::remove(path);
}

TEST_CASE("sha256Text gives the standard SHA-256 vectors", "[plan][bundle]") {
  CHECK(x7::ptp::sha256Text("") ==
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  CHECK(x7::ptp::sha256Text("abc") ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  // 56 bytes: the padding takes a second block.
  CHECK(x7::ptp::sha256Text(
            "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  // Bytes, not a C string: an embedded NUL counts.
  CHECK(x7::ptp::sha256Text(std::string("a\0b", 3)) !=
        x7::ptp::sha256Text("a"));
  // The same digest as the file holding the text.
  const fs::path path = scratchDir() / "text.bin";
  fs::remove(path);
  const std::string text = "rtctrl-x7-follow-reference-signature 1\n";
  writeBytes(path, text);
  CHECK(x7::ptp::sha256Text(text) == x7::ptp::sha256File(path));
  fs::remove(path);
}

TEST_CASE("sha256File throws for a missing file", "[plan][bundle]") {
  const fs::path path = scratchDir() / "missing.bin";
  fs::remove(path);
  REQUIRE_FALSE(fs::exists(path));
  CHECK_THROWS_AS(x7::ptp::sha256File(path), std::runtime_error);
}
