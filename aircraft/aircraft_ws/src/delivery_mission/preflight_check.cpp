#include "delivery_mission/preflight_check.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleStatus;

namespace delivery_mission
{

// ── PreflightResult::print ────────────────────────────────────────────────────
void PreflightResult::print() const
{
  auto yn = [](bool v) { return v ? "\033[32m OK \033[0m" : "\033[31m FAIL\033[0m"; };

  RCLCPP_INFO(rclcpp::get_logger("preflight"),
    "\n"
    "╔══════════════════════════════╗\n"
    "║     PRE-FLIGHT REPORT        ║\n"
    "╠══════════════════════════════╣\n"
    "║  Vehicle (PX4 arm check) [%s]║\n"
    "║  GPS fix / satellites    [%s]║\n"
    "║  Battery level           [%s]║\n"
    "╠══════════════════════════════╣\n"
    "║  DECISION: %-19s             ║\n"
    "╚══════════════════════════════╝",
    yn(vehicle_ok), yn(gps_ok), yn(battery_ok),
    go ? "\033[32mGO\033[0m" : "\033[31mNO-GO\033[0m");
}

// ── Constructor ───────────────────────────────────────────────────────────────
PreflightCheck::PreflightCheck(const rclcpp::NodeOptions & options)
: Node("preflight_check", options)
{
  // AAS PX4 namespace is /Drone1 by default (overridable via __ns remapping)
  const auto qos = rclcpp::QoS(10).best_effort();

  sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "fmu/out/vehicle_status", qos,
    [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) { cb_status(msg); });

  sub_gps_ = create_subscription<px4_msgs::msg::VehicleGpsPosition>(
    "fmu/out/vehicle_gps_position", qos,
    [this](const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg) { cb_gps(msg); });

  sub_battery_ = create_subscription<px4_msgs::msg::BatteryStatus>(
    "fmu/out/battery_status", qos,
    [this](const px4_msgs::msg::BatteryStatus::SharedPtr msg) { cb_battery(msg); });

  RCLCPP_INFO(get_logger(), "PreflightCheck node started — waiting for topics...");
}

// ── run() — spin until all data received, then evaluate ──────────────────────
PreflightResult PreflightCheck::run()
{
  // Spin until all three topics have delivered at least one message
  rclcpp::Rate rate(10);  // 10 Hz polling
  while (rclcpp::ok() && !(got_status_ && got_gps_ && got_battery_)) {
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }

  PreflightResult result;
  result.vehicle_ok  = arming_check_ok_ && nav_state_ok_;
  result.gps_ok      = (gps_satellites_ >= MIN_GPS_SATELLITES) && (gps_eph_ <= MIN_GPS_EPH);
  result.battery_ok  = (battery_percent_ >= MIN_BATTERY_PERCENT);
  result.go          = result.vehicle_ok && result.gps_ok && result.battery_ok;

  // Detailed log before summary
  RCLCPP_INFO(get_logger(),
    "Details → arm_check: %s | nav_state_ok: %s | sats: %d | eph: %.1fm | battery: %.1f%%",
    arming_check_ok_ ? "pass" : "fail",
    nav_state_ok_    ? "pass" : "fail",
    gps_satellites_,
    static_cast<double>(gps_eph_),
    static_cast<double>(battery_percent_));

  result.print();
  return result;
}

// ── Callbacks ─────────────────────────────────────────────────────────────────
void PreflightCheck::cb_status(const VehicleStatus::SharedPtr msg)
{
  if (got_status_) return;  // only need first reading

  // arming_check_compliant was added in px4_msgs to signal all arm checks passed
  arming_check_ok_ = (msg->pre_flight_checks_pass);

  // nav_state 0 = MANUAL, just verify the FMU is responding and not in failsafe
  nav_state_ok_ = (msg->nav_state != VehicleStatus::NAVIGATION_STATE_TERMINATION) &&
                  (msg->nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD);
                  // not already in offboard — clean slate

  got_status_ = true;
  RCLCPP_DEBUG(get_logger(), "[status] pre_flight_checks_pass=%d nav_state=%d",
    msg->pre_flight_checks_pass, msg->nav_state);
}

void PreflightCheck::cb_gps(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg)
{
  if (got_gps_) return;

  gps_satellites_ = static_cast<int>(msg->satellites_used);
  gps_eph_        = msg->eph;   // estimated horizontal position error in metres

  got_gps_ = true;
  RCLCPP_DEBUG(get_logger(), "[gps] sats=%d eph=%.1f", gps_satellites_,
    static_cast<double>(gps_eph_));
}

void PreflightCheck::cb_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  if (got_battery_) return;

  battery_percent_ = msg->remaining * 100.0f;  // PX4 gives 0.0–1.0

  got_battery_ = true;
  RCLCPP_DEBUG(get_logger(), "[battery] remaining=%.1f%%",
    static_cast<double>(battery_percent_));
}

} // namespace delivery_mission

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<delivery_mission::PreflightCheck>();
  auto result = node->run();

  rclcpp::shutdown();
  return result.go ? EXIT_SUCCESS : EXIT_FAILURE;
}