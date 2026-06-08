#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>

namespace delivery_mission
{

// Minimum thresholds for a GO decision
constexpr float MIN_BATTERY_PERCENT  = 20.0f;   // %
constexpr int   MIN_GPS_SATELLITES   = 6;        // satellites used
constexpr float MIN_GPS_EPH          = 5.0f;     // horizontal position uncertainty (m)

struct PreflightResult
{
  bool vehicle_ok   = false;   // PX4 pre-arm checks passed
  bool gps_ok       = false;   // GPS fix with enough satellites
  bool battery_ok   = false;   // battery above minimum
  bool go           = false;   // overall GO / NO-GO

  void print() const;
};

class PreflightCheck : public rclcpp::Node
{
public:
  explicit PreflightCheck(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Blocks until all three topics have been received once, then returns result.
  PreflightResult run();

private:
  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr       sub_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr  sub_gps_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr       sub_battery_;

  // Latest message flags
  bool got_status_  = false;
  bool got_gps_     = false;
  bool got_battery_ = false;

  // Stored results
  bool arming_check_ok_  = false;
  bool nav_state_ok_     = false;
  int  gps_satellites_   = 0;
  float gps_eph_         = 999.0f;
  float battery_percent_ = 0.0f;

  void cb_status (const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void cb_gps    (const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg);
  void cb_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
};

} // namespace delivery_mission