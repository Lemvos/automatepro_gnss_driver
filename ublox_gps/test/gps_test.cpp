// Unit tests for ublox_gps::Gps guards that are only reachable on the teardown
// and failed-bring-up boundaries, where the object exists but its I/O worker
// does not.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/gps.hpp>

// A Gps exists from the moment on_configure() constructs it, but worker_ is only
// set once the serial port opens. A configure that failed on a missing device
// therefore leaves a live Gps with no worker, and /rtcm corrections keep
// arriving because that subscription outlives the lifecycle transitions.
// Dereferencing worker_ there segfaulted the process (SIGSEGV, exit -11) and
// took the recovery ladder with it.
TEST(GpsTest, SendRtcmWithoutWorkerDoesNotCrash) {
  ublox_gps::Gps gps(0, rclcpp::get_logger("gps_test"));
  ASSERT_FALSE(gps.isInitialized()) << "a freshly constructed Gps has no worker";

  const std::vector<uint8_t> rtcm{0xd3, 0x00, 0x13, 0x3e, 0xd7, 0xd3};
  EXPECT_FALSE(gps.sendRtcm(rtcm))
      << "sendRtcm must report failure rather than dereference a null worker";
}

// The same boundary reached through the other senders, which have always been
// guarded; asserted here so the three stay consistent.
TEST(GpsTest, PollWithoutWorkerReportsFailure) {
  ublox_gps::Gps gps(0, rclcpp::get_logger("gps_test"));
  ASSERT_FALSE(gps.isInitialized());

  EXPECT_FALSE(gps.poll(0x0a, 0x04, std::vector<uint8_t>{}))
      << "poll must report failure when there is no worker";
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
