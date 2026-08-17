// Unit tests for the legacy GNSS comms watchdog (ublox_gps/watchdog.hpp).
// The driver no longer uses this class: the comms-liveness check now runs as a
// wall timer in UbloxNode::watchdogCheck. These tests are retained only to keep
// the legacy header verified while it remains in tree.

#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "ublox_gps/watchdog.hpp"

using namespace std::chrono_literals;

namespace {

// Spin until `pred` is true or `budget` elapses; lets timing tests assert on an
// outcome without sleeping for a fixed (flaky) duration.
template <typename Pred>
bool waitFor(Pred pred, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  return pred();
}

}  // namespace

// No data within the timeout must request recovery.
TEST(WatchdogTest, DropoutTriggersRecovery) {
  std::atomic<int> fired{0};

  Watchdog wd;
  wd.set_timeout(80ms);
  wd.set_check_interval(10ms);
  wd.set_callback([&fired]() { fired.fetch_add(1); });
  wd.start();

  EXPECT_TRUE(waitFor([&fired]() { return fired.load() >= 1; }, 1000ms))
      << "watchdog did not request recovery after the timeout elapsed";
}

// A live link (reset within the timeout) must not request recovery.
TEST(WatchdogTest, LiveLinkDoesNotTriggerRecovery) {
  std::atomic<int> fired{0};

  Watchdog wd;
  wd.set_timeout(200ms);
  wd.set_check_interval(10ms);
  wd.set_callback([&fired]() { fired.fetch_add(1); });
  wd.start();

  for (int i = 0; i < 30; ++i) {        // reset within the timeout for ~600 ms
    wd.reset();
    std::this_thread::sleep_for(20ms);
  }
  wd.stop();

  EXPECT_EQ(fired.load(), 0)
      << "watchdog requested recovery while data was still flowing";
}

// A persistent outage re-fires every interval; the idempotent flag keeps each
// pass O(1) (the design that replaced the old unbounded recursion).
TEST(WatchdogTest, PersistentOutageReFiresBounded) {
  std::atomic<int> fired{0};
  std::atomic<bool> recovery_requested{false};

  Watchdog wd;
  wd.set_timeout(60ms);
  wd.set_check_interval(10ms);
  wd.set_callback([&]() {
    fired.fetch_add(1);
    recovery_requested.store(true);
  });
  wd.start();

  EXPECT_TRUE(waitFor([&fired]() { return fired.load() >= 3; }, 1000ms))
      << "watchdog stopped re-requesting recovery during a sustained outage";
  EXPECT_TRUE(recovery_requested.load());

  // When the link recovers, resets keep the watchdog satisfied and it stops.
  const int before = fired.load();
  for (int i = 0; i < 20; ++i) {
    wd.reset();
    std::this_thread::sleep_for(10ms);
  }
  wd.stop();
  EXPECT_LE(fired.load() - before, 2)   // a couple may race the first resets
      << "watchdog kept firing after the link recovered";
}

// Timeout / check-interval updated concurrently with the watchdog thread must
// not race or tear (the fields are std::atomic).
TEST(WatchdogTest, ConcurrentTimeoutUpdatesAreRaceFree) {
  std::atomic<int> fired{0};

  Watchdog wd;
  wd.set_timeout(50ms);
  wd.set_check_interval(2ms);
  wd.set_callback([&fired]() { fired.fetch_add(1); });
  wd.start();

  std::atomic<bool> stop{false};
  std::thread writer([&]() {
    while (!stop.load()) {
      wd.set_timeout(40ms);
      wd.set_check_interval(3ms);
      wd.reset();
      wd.set_timeout(60ms);
      wd.set_check_interval(2ms);
    }
  });

  std::this_thread::sleep_for(300ms);
  stop.store(true);
  writer.join();
  wd.stop();

  // The watchdog must still report a value the writer actually set, intact.
  const auto t = wd.get_timeout();
  EXPECT_TRUE(t == 40ms || t == 60ms) << "torn read of timeout_: " << t.count();
}
