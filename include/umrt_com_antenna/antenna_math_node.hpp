#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <iostream>
#include <string>
#include <chrono>
#include <numbers>
#include <cmath>
#include <mutex>
#include <atomic>

namespace umrt_com_antenna
{

class AntennaPID: public rclcpp::Node
{
public:

  /*
    * AntennaPID:
    *  The constructor for the antenna pid class
    * 
    * Builds a new antenna pid and setups the required variables
    */

  AntennaPID();
  ~AntennaPID() override;

private:

  // VARIABLES
  std::mutex data_mutex_;

  std::atomic<float> heading{-1.0f};
  std::atomic<long long> lastOverrideReceived{-1};

  float lastRoverLon = -9999;
  float lastRoverLat = -9999;
  float lastAntennaLon = -9999;
  float lastAntennaLat = -9999;

  rclcpp::CallbackGroup::SharedPtr cb_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_bs;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_antenna_pos;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_rover_pos;

  // FUNCTION SIGNATURES
  void bs_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void new_rover_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void new_antenna_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  double rotation_based_on_cord(const double &lon2, const double &lat2, const double &lon1, const double &lat1);

  void send_msg();

};

}  // namespace umrt_com_antenna