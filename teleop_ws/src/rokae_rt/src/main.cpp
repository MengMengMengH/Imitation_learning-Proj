#include <iostream>
#include <iostream>
#include <cmath>
#include <thread>
#include <mutex> // 用于线程同步
#include <array>

#include "rokae_rt/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using namespace std::chrono_literals;
using namespace rokae;

class rt_RobotCtrlNode :  public rclcpp::Node
{
public:
    rt_RobotCtrlNode() : Node("rt_robot_control_node")
    {
        // 初始化rokae机器人
        std::error_code ec;
        try
        {
            std::string robot_ip = "192.168.0.160";
            std::string local_ip = "192.168.0.100";
            robot_.connectToRobot(robot_ip, local_ip);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        robot_.setOperateMode(OperateMode::automatic, ec);
        robot_.setRtNetworkTolerance(20, ec);
        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setPowerState(true,ec);
        
        //订阅节点
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .deadline(rclcpp::Duration(1ms));

        joint_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos, 
            std::bind(&rt_RobotCtrlNode::jointPositionCallback, this, std::placeholders::_1)
        );
        // 初始化关节位置
        try
        {
            RCLCPP_INFO(this->get_logger(), "Initializing robot joint positions to zero.");
            motion_controller_ = robot_.getRtMotionController().lock();
            robot_.startReceiveRobotState(std::chrono::milliseconds(1),{RtSupportedFields::jointPos_m});

            std::array<double, 7> cur_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
            
            std::cout << "Current joint positions: ";
            for (double value : cur_pos) {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            PowerState state = robot_.powerState(ec);
            std::string state_str;
            switch (state)
            {
                case PowerState::on:
                    state_str = "上电";
                    break;
                case PowerState::off:
                    state_str = "下电";
                    break;
                case PowerState::unknown:
                    state_str = "未知";
                    break;
                case PowerState::estop:
                    state_str = "急停";
                    break;
                case PowerState::gstop:
                    state_str = "安全门打开";
                    break;
                default:
                    state_str = "无效状态";
                    break;
            }
            std::cout << "Robot power state: " << state_str << std::endl;

            motion_controller_->MoveJ(0.5,cur_pos,zero_pos);
            RCLCPP_INFO(this->get_logger(), "Robot joint positions initialized to zero.");
        }
        catch(std::exception &e)
        {
            std::cerr << e.what();
        }

        std::function<JointPosition()> rokae_callback = [&,this] ()
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            // std::array<double, 7> current_pos = robot_.jointPos(ec);
            current_target_joint_pos_ = rec_joint_positions_;
            if(current_target_joint_pos_ == zero_pos && !init_joint_pos_set_ && has_received_joint_positions_) 
            {init_joint_pos_set_ = true;}

            JointPosition cmd;
            std::cout << init_joint_pos_set_<< std::endl;
            if(init_joint_pos_set_)
            {
                cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
            }
            else
            {
                cmd.joints = std::vector<double>(zero_pos.begin(), zero_pos.end());
            }
            return cmd;
        };

        motion_controller_->setControlLoop(rokae_callback);
        motion_controller_->startMove(RtControllerMode::jointPosition);
        control_thread_ = std::thread([this]() 
        {
            this->motion_controller_->startLoop(true);
        });
    };

    ~rt_RobotCtrlNode()
    {
        if(control_thread_.joinable())
        {
            control_thread_.join();
        }
        std::error_code ec;
        robot_.setPowerState(false, ec);
        std::cout<< "Robot power off." << std::endl;
    }

private:

    void jointPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() != 7)
        {
            RCLCPP_ERROR(this->get_logger(), "Received joint positions size is not 7.");
            return;
        }
        std::lock_guard<std::mutex> lock(joint_positions_mutex_);
        if(!msg->data.empty())
        {
            for(size_t i = 0; i < 7; ++i)
            {
                rec_joint_positions_[i] = msg->data[i];
            }
            has_received_joint_positions_ = true;
        }
    }

    rokae::xMateErProRobot robot_;

    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;
    
    const std::array<double, 7> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //线程共享数据
    std::array<double, 7> rec_joint_positions_;
    std::mutex joint_positions_mutex_;

    bool init_joint_pos_set_ = false;
    bool has_received_joint_positions_ = false;

    std::array<double, 7> current_target_joint_pos_ ;
    std::thread control_thread_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rt_RobotCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}