#include <catch2/catch_test_macros.hpp>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <roki/rk_chain.h>
#include <toml++/toml.hpp>

#include <cstring>

#include "rtctrl/version.hpp"

TEST_CASE("roki chain initializes and destroys cleanly", "[smoke]") {
  rkChain chain;
  rkChainInit(&chain);
  CHECK(rkChainLinkNum(&chain) == 0);
  rkChainDestroy(&chain);
}

TEST_CASE("DynamixelSDK provides a protocol 2.0 packet handler", "[smoke]") {
  auto* packet = dynamixel::PacketHandler::getPacketHandler(2.0);
  REQUIRE(packet != nullptr);
  CHECK(packet->getProtocolVersion() == 2.0f);
}

TEST_CASE("toml++ parses an array-of-tables config shape", "[smoke]") {
  const auto tbl = toml::parse(R"(
    [[joint]]
    name = "crane_x7_wrist_joint"
    id = 8
  )");
  const auto* joints = tbl["joint"].as_array();
  REQUIRE(joints != nullptr);
  REQUIRE(joints->size() == 1);
  CHECK((*joints)[0].as_table()->at("id").value<int>() == 8);
}

TEST_CASE("rtctrl reports a version", "[smoke]") {
  REQUIRE(rtctrl::version() != nullptr);
  CHECK(std::strlen(rtctrl::version()) > 0);
}
