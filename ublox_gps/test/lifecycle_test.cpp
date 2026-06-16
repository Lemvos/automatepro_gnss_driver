// Lifecycle transition tests for UbloxNode. Drives the node in-process and
// asserts the resulting state at each transition. The device-independent cases
// run anywhere; the full ladder needs a receiver (UBLOX_TEST_DEVICE).
// See docs/error-handling-and-recovery.md.

#include <cstdlib>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <ublox_gps/node.hpp>

using lifecycle_msgs::msg::State;

namespace {

// A non-existent device path makes configure() fail deterministically at
// serial open, regardless of what is plugged into the test host.
std::shared_ptr<ublox_node::UbloxNode> makeNode(const std::string & device) {
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("device", device);
  return std::make_shared<ublox_node::UbloxNode>(opts);
}

constexpr const char * kAbsentDevice = "/dev/ublox_does_not_exist";

}  // namespace

// A freshly constructed node is UNCONFIGURED and touches no hardware.
TEST(LifecycleTest, ConstructsUnconfigured) {
  auto node = makeNode(kAbsentDevice);
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// Fail-safe: with no receiver, configure() must leave the node UNCONFIGURED.
TEST(LifecycleTest, ConfigureWithoutDeviceFailsSafe) {
  auto node = makeNode(kAbsentDevice);

  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED)
      << "a failed configure must leave the node UNCONFIGURED, not partially up";

  // shutdown() is qualified: UbloxNode::shutdown() hides the transition.
  EXPECT_EQ(node->rclcpp_lifecycle::LifecycleNode::shutdown().id(),
            State::PRIMARY_STATE_FINALIZED);
}

// Shutdown straight from UNCONFIGURED (gps_ null) must not crash
// (regression test for the shutdown() null-deref guard).
TEST(LifecycleTest, ShutdownFromUnconfigured) {
  auto node = makeNode(kAbsentDevice);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node->rclcpp_lifecycle::LifecycleNode::shutdown().id(),
            State::PRIMARY_STATE_FINALIZED);
}

// Full transition ladder against a real receiver (mirrors the old .sh).
// Skipped unless UBLOX_TEST_DEVICE points at an attached u-blox device.
TEST(LifecycleTest, FullTransitionSequenceOnDevice) {
  const char * dev = std::getenv("UBLOX_TEST_DEVICE");
  if (dev == nullptr || dev[0] == '\0') {
    GTEST_SKIP() << "set UBLOX_TEST_DEVICE=/dev/<receiver> to run the HIL ladder";
  }
  auto node = makeNode(dev);

  ASSERT_EQ(node->configure().id(),  State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),   State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(),    State::PRIMARY_STATE_UNCONFIGURED);
  // reconfigure / reactivate (the part the .sh exercised twice)
  ASSERT_EQ(node->configure().id(),  State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),   State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),   State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node->rclcpp_lifecycle::LifecycleNode::shutdown().id(),
            State::PRIMARY_STATE_FINALIZED);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
