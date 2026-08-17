// Standalone entry point for the u-blox lifecycle node.
//
// The node owns its lifecycle: rclcpp's signal handlers are disabled and this
// process installs its own SIGINT/SIGTERM handlers. A signal only sets an
// async-signal-safe stop flag; a watcher thread breaks executor.spin() with
// cancel(), keeping the ROS context valid so the single finalize() teardown
// ladder (deactivate -> cleanup -> shutdown) can run cleanly before
// rclcpp::shutdown().
//
// Structure follows automatepro_ws_template/src/main_lifecycle.cpp; keep the
// two in step. The only deliberate divergence is the qualified call to
// LifecycleNode::shutdown() in finalize(), which UbloxNode::shutdown() hides.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <memory>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <ublox_gps/node.hpp>

namespace
{

// Set by the signal handler, polled by main() and the watcher thread. Must be
// lock-free to be safe to touch from a signal handler.
std::atomic<bool> g_stop_requested{false};
static_assert(
  std::atomic<bool>::is_always_lock_free,
  "std::atomic<bool> must be lock-free to be async-signal-safe");

// How often the watcher thread polls the stop flag while spinning.
constexpr std::chrono::milliseconds kStopPollInterval{100};
// Short back-off while waiting for the executor to start spinning.
constexpr std::chrono::milliseconds kSpinStartPollInterval{1};

// Async-signal-safe: the first signal requests a graceful stop; a second forces
// an immediate exit. No logging, allocation, or library calls here.
void handle_stop_signal([[maybe_unused]] int signum)
{
  if (g_stop_requested.exchange(true)) {
    std::_Exit(EXIT_FAILURE);
  }
}

// Single checked teardown ladder run on every post-construction exit path.
// noexcept because the lifecycle convenience methods carry no no-throw contract
// and this must never propagate out of main().
void finalize(const std::shared_ptr<ublox_node::UbloxNode> & node) noexcept
{
  try {
    if (!node || !rclcpp::ok()) {
      return;
    }

    auto state_id = node->get_current_state().id();

    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Deactivating node...");
      state_id = node->deactivate().id();
    }

    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Cleaning up node...");
      state_id = node->cleanup().id();
      // Cleanup error-processes into FINALIZED instead of UNCONFIGURED on failure.
      if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Cleanup failed; node finalized via error.");
      }
    }

    // shutdown() is valid from ACTIVE, INACTIVE, or UNCONFIGURED, so run it from
    // any non-finalized state -- including when on_deactivate returned FAILURE and
    // left the node ACTIVE -- to guarantee on_shutdown runs before destruction.
    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
      state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down node...");
      // UbloxNode::shutdown() closes the serial port and hides the inherited
      // transition, so the base name is qualified here.
      state_id = node->rclcpp_lifecycle::LifecycleNode::shutdown().id();
    }

    if (state_id != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
      RCLCPP_ERROR(
        rclcpp::get_logger("main"), "Teardown ended in state %d instead of finalized.", state_id);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception during teardown: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown exception during teardown.");
  }
}

// RAII guard that always requests a stop and joins the watcher thread on scope
// exit, including exception unwinding out of spin(). Both flags must be set
// before join(): the stop flag releases the watcher's first wait, and the
// spin-finished flag releases its cancel-before-spin guard, which would
// otherwise wait forever for a spin that has already unwound. Joining before
// std::thread's destructor runs avoids std::terminate() on a joinable thread.
struct WatcherGuard
{
  std::thread & watcher;
  std::atomic<bool> & spin_finished;

  ~WatcherGuard() noexcept
  {
    spin_finished.store(true);
    g_stop_requested.store(true);
    if (watcher.joinable()) {
      watcher.join();
    }
  }
};

}  // namespace

int main(int argc, char * argv[])
{
  // Own the lifecycle: disable rclcpp's signal handlers and install our own so
  // termination runs the teardown ladder below instead of rclcpp's default
  // context shutdown.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handle_stop_signal);
  std::signal(SIGTERM, handle_stop_signal);

  std::shared_ptr<ublox_node::UbloxNode> node;
  int exit_code = EXIT_SUCCESS;

  // Any throw from construction, bring-up, add_node, or spin() must still reach
  // the single finalize() ladder below, so the whole run is wrapped and the
  // catch only records the failure.
  try {
    node = std::make_shared<ublox_node::UbloxNode>(rclcpp::NodeOptions());

    // Bring-up: configure -> activate, checking the resulting state each step and
    // honouring a stop that arrives during bring-up.
    if (g_stop_requested.load()) {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Stop requested before bring-up. Shutting down...");
    } else if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_FATAL(rclcpp::get_logger("main"), "Node failed to configure; exiting.");
      exit_code = EXIT_FAILURE;
    } else if (g_stop_requested.load()) {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Stop requested during configure. Shutting down...");
    } else if (node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_FATAL(rclcpp::get_logger("main"), "Node failed to activate; exiting.");
      exit_code = EXIT_FAILURE;
    } else {
      rclcpp::executors::SingleThreadedExecutor executor;
      // Lifecycle nodes expose the base interface so the executor services both
      // regular and lifecycle-related callbacks.
      executor.add_node(node->get_node_base_interface());

      // Set once spin() has returned, normally or by exception. spin() clears
      // its spinning flag while unwinding, so without this the watcher's
      // cancel-before-spin guard below would wait for a spin that never resumes
      // and the join() in WatcherGuard would never return.
      std::atomic<bool> spin_finished{false};

      // The signal handler cannot cancel the executor directly (not
      // async-signal-safe), so a watcher thread breaks spin() on the stop flag.
      // is_spinning() guards the cancel-before-spin race.
      std::thread watcher(
        [&executor, &spin_finished]() {
          while (rclcpp::ok() && !g_stop_requested.load()) {
            std::this_thread::sleep_for(kStopPollInterval);
          }
          while (rclcpp::ok() && !spin_finished.load() && !executor.is_spinning()) {
            std::this_thread::sleep_for(kSpinStartPollInterval);
          }
          executor.cancel();
        });
      // Guarantees the watcher is stopped and joined on every exit from this
      // scope -- normal return or an exception out of spin().
      WatcherGuard watcher_guard{watcher, spin_finished};

      RCLCPP_INFO(rclcpp::get_logger("main"), "Node is running.");
      executor.spin();

      RCLCPP_INFO(rclcpp::get_logger("main"), "Stop received. Shutting down...");
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception during bring-up or spin: %s", e.what());
    exit_code = EXIT_FAILURE;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Unknown exception during bring-up or spin.");
    exit_code = EXIT_FAILURE;
  }

  finalize(node);
  node.reset();

  RCLCPP_INFO(rclcpp::get_logger("main"), "Node has shut down gracefully.");
  rclcpp::shutdown();
  return exit_code;
}
