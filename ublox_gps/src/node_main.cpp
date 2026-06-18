#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <ublox_gps/node.hpp>

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;

namespace {
  // Set in the signal handler, read in the watcher thread.
  static_assert(std::atomic<bool>::is_always_lock_free);
  std::atomic<bool> g_stop{false};
  // Async-signal-safe: only an atomic store and (on a second signal) _Exit.
  void on_signal(int) {
    // First signal: request graceful stop. Second: force quit.
    if (g_stop.exchange(true, std::memory_order_relaxed))
      std::_Exit(1);
  }
}

int main(int argc, char ** argv)
{
  // Drive the lifecycle ourselves; rclcpp's own signal handlers are disabled.
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,  on_signal);
  std::signal(SIGTERM, on_signal);

  rclcpp::executors::SingleThreadedExecutor exec;
  std::shared_ptr<ublox_node::UbloxNode> node;

  // One checked teardown ladder, run on EVERY post-construction exit (bring-up
  // failure, exception, or normal stop) so on_cleanup/on_shutdown always run and
  // every result is checked. No divergent ad-hoc teardown. Safe to call from any
  // post-construction state; only ever invoked once node is non-null.
  auto finalize = [&] {
    auto id = [&]{ return node->get_current_state().id(); };
    if (!rclcpp::ok() || id() == State::PRIMARY_STATE_FINALIZED
                      || id() == State::PRIMARY_STATE_UNKNOWN) {
      RCLCPP_ERROR(node->get_logger(),
        "teardown skipped: context down or node not in a teardownable state");
      return;
    }
    if (id() == State::PRIMARY_STATE_ACTIVE
        && node->deactivate().id() != State::PRIMARY_STATE_INACTIVE)
      RCLCPP_ERROR(node->get_logger(), "deactivate failed: fail-safe did NOT run");
    if (id() == State::PRIMARY_STATE_INACTIVE
        && node->cleanup().id() != State::PRIMARY_STATE_UNCONFIGURED)
      RCLCPP_ERROR(node->get_logger(), "cleanup failed: resources may be leaked");
    // LifecycleNode::shutdown() is qualified: UbloxNode::shutdown() hides it.
    if (id() != State::PRIMARY_STATE_FINALIZED && id() != State::PRIMARY_STATE_UNKNOWN
        && node->rclcpp_lifecycle::LifecycleNode::shutdown().id()
             != State::PRIMARY_STATE_FINALIZED)
      RCLCPP_ERROR(node->get_logger(), "shutdown did not reach finalized");
  };

  // Bring-up: construction and each transition are checked, not assumed.
  // Node is destroyed before rclcpp::shutdown() on every exit path.
  try {
    node = std::make_shared<ublox_node::UbloxNode>(rclcpp::NodeOptions());
    exec.add_node(node->get_node_base_interface());

    if (node->configure().id() != State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "configure failed");
      finalize(); node.reset(); rclcpp::shutdown(); return 1;
    }
    // Don't activate if a stop was already requested during bring-up.
    if (g_stop.load(std::memory_order_relaxed)) {
      RCLCPP_WARN(node->get_logger(), "stop requested during bring-up; not activating");
    } else if (node->activate().id() != State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "activate failed");
      finalize(); node.reset(); rclcpp::shutdown(); return 1;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ublox_gps_node"), "bring-up failed: %s", e.what());
    if (node) finalize();                     // fail-safe still runs
    node.reset(); rclcpp::shutdown(); return 1;
  }

  // Run until a stop is requested; the watcher cancels the spin.
  if (node->get_current_state().id() == State::PRIMARY_STATE_ACTIVE
      && !g_stop.load(std::memory_order_relaxed)) {
    RCLCPP_INFO(node->get_logger(), "Node running.");
    std::thread watcher([&exec]() {
      while (rclcpp::ok() && !g_stop.load(std::memory_order_relaxed))
        std::this_thread::sleep_for(100ms);
      while (rclcpp::ok() && !exec.is_spinning())
        std::this_thread::sleep_for(5ms);               // avoid cancel-before-spin race
      exec.cancel();
    });
    exec.spin();
    watcher.join();
  }

  finalize();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
