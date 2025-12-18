#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <mutex> // 用于线程同步


using namespace std::chrono_literals;

class topicTestNode :  public rclcpp::Node
{
public:
    topicTestNode() : Node("topic_test_node")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .deadline(rclcpp::Duration(1ms));

        joint_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos, 
            std::bind(&topicTestNode::jointPositionCallback, this, std::placeholders::_1)
        );
        
        last_positions_.resize(num_joints, 0.0f);
        last_velocities_.resize(num_joints, 0.0f);
        last_accelerations_.resize(num_joints, 0.0f);
    }

private:
    void jointPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        rclcpp::Time current_time = this->get_clock()->now();
        if (msg->data.size() != num_joints)
        {
            RCLCPP_ERROR(this->get_logger(), "Received message with unexpected size: %zu (expected %zu)", msg->data.size(), num_joints);
            return;
        }

        if (is_first_msg_)
        {
            last_time_ = current_time;
            last_positions_ = msg->data;
            // last_velocities_ 和 last_accelerations_ 已在构造函数中初始化为0
            is_first_msg_ = false;
            RCLCPP_INFO(this->get_logger(), "First message received. Initializing state.");
            return;
        }
        
        //debug:检测消息间隔
        static rclcpp::Time last_msg_time = this->get_clock()->now();
        rclcpp::Time current_msg_time = this->get_clock()->now();
        double interval_ms = (current_msg_time - last_msg_time).seconds() * 1000.0;
        last_msg_time = current_msg_time;
        if (interval_ms > 1.5) 
        {
            RCLCPP_WARN(this->get_logger(), "Large message interval detected: %.4f ms", interval_ms);
        }

        static int burst_count = 0;
        if (interval_ms > 3.0) {  // 超过3ms视为“积压”
            burst_count = 0;
            RCLCPP_WARN(this->get_logger(),
                "⚠️ Possible delivery gap detected: %.3f ms without new data", interval_ms);
        } else if (interval_ms < 0.3) {  // 小于0.3ms说明可能是批量交付
            burst_count++;
            if (burst_count >= 3) {
                RCLCPP_ERROR(this->get_logger(),
                    "🚨 DDS burst delivery detected! %d messages delivered within %.3f ms",
                    burst_count, interval_ms);
                burst_count = 0;
            }
        }

        current_positions_ = msg->data;

        for (size_t i = 0; i < num_joints; ++i)
        {
            current_velocities[i] = (current_positions_[i] - last_positions_[i]) / dt;
            current_accelerations[i] = (current_velocities[i] - last_velocities_[i]) / dt;
            current_jerks[i] = (current_accelerations[i] - last_accelerations_[i]) / dt;
        }
        check_limits(current_velocities, joint_velocity_limits_, "Velocity");
        check_limits(current_accelerations, joint_acceleration_limits_, "Acceleration");
        check_limits(current_jerks, joint_jerk_limits_, "Jerk");
        last_positions_ = current_positions_;
        last_velocities_ = current_velocities;
        last_accelerations_ = current_accelerations;
    }

    void check_limits(const std::vector<float>& values, 
                      const std::vector<double>& limits, 
                      const std::string& limit_name)
    {
        for (size_t i = 0; i < values.size(); ++i)
        {
            // 检查绝对值是否超限
            if (std::abs(values[i]) > limits[i])
            {
                // 使用节流日志 (THROTTLE)
                // 同样的消息（同一个关节、同一个限制）在1000ms内最多只打印一次
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    100, // 节流周期：100 毫秒
                    "PHYSICAL LIMIT VIOLATION! Joint[%zu] %s Limit Exceeded! \n"
                    "    Value: %.4f, Limit: %.4f (dt=%.4fs)",
                    i,
                    limit_name.c_str(),
                    values[i],
                    limits[i],
                    dt
                );
            }
            if (limit_name == "Velocity" && std::abs(values[i]) > limits[i])
            {
                RCLCPP_ERROR_THROTTLE( // 改用 ERROR，以便更容易注意到
                    this->get_logger(),
                    *this->get_clock(),
                    100,
                    "FATAL JUMP DETECTED! Joint[%zu] VIO: %.4f | Prev Pos: %.10f, Current Pos: %.10f",
                    i,
                    values[i],
                    last_positions_[i],     // 上一个位置 (Pn-1)
                    current_positions_[i]    // 当前位置 (Pn)
                );
            }
            if (limit_name == "Acceleration" && std::abs(values[i]) > limits[i])
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    100,
                    "FATAL JUMP DETECTED! Joint[%zu] AIO: %.4f | Prev vel: %.10f, Current vel: %.10f",
                    i,
                    values[i],
                    last_velocities_[i],   
                    current_velocities[i] 
                );
            }

            if (limit_name == "Jerk" && std::abs(values[i]) > limits[i])
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    100,
                    "FATAL JUMP DETECTED! Joint[%zu] JIO: %.4f | Prev acc: %.10f, Current acc: %.10f",
                    i,
                    values[i],
                    last_accelerations_[i],    
                    current_accelerations[i]   
                );
            }
        }
    }


    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;
    rclcpp::Time last_time_;

    bool is_first_msg_ = true;
    const size_t num_joints = 7;
    const double dt = 0.001;

    std::vector<float> last_positions_ =  {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<float> last_velocities_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<float> last_accelerations_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<float> current_positions_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    std::vector<float> current_velocities = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<float> current_accelerations = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<float> current_jerks = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};


    std::vector<double> joint_velocity_limits_ = { 2.175,  2.175,  2.175,  2.175,  2.610,  2.610,  2.610};
    std::vector<double> joint_acceleration_limits_ = { 15,  7.5,  10,  10,  15,  15,  20};
    std::vector<double> joint_jerk_limits_ = { 5000,  3500,  5000,  5000,  7500,  7500,  7500};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<topicTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}