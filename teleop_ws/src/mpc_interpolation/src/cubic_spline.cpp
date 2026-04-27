#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class RokaeControlPublisher : public rclcpp::Node {
public:
  RokaeControlPublisher()
  : Node("rokae_control_node"),
    start_time_(std::chrono::steady_clock::now())
  {
    amplitudes_  = {0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    frequencies_ = {0.5, 0.5,0.5, 0.5,0.5, 0.5, 0.5};
    phases_      = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .deadline(rclcpp::Duration(1ms));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "rokae_control_joints", qos);

    timer_ = this->create_wall_timer(
        20ms, std::bind(&RokaeControlPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Rokae 50Hz node started. First 5s output zero.");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.resize(7, 0.0f);

    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - start_time_).count();

    // 前 5 秒：保持全零
    if (t < 5.0) {
      publisher_->publish(message);
      return;
    }

    // 5 秒后：开始正弦
    double ts = t - 5.0;  // 正弦时间，从 0 开始

    for (size_t i = 0; i < 7; ++i) {
      double angle =
        amplitudes_[i] *
        (std::cos(2.0 * M_PI * frequencies_[i] * ts + phases_[i])-1);

      message.data[i] = static_cast<float>(angle);
    }

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

  std::chrono::steady_clock::time_point start_time_;

  std::vector<double> amplitudes_;
  std::vector<double> frequencies_;
  std::vector<double> phases_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RokaeControlPublisher>());
  rclcpp::shutdown();
  return 0;
}
