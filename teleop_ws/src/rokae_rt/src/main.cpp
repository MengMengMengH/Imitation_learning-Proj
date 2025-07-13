#include <iostream>
#include <cmath>
#include <thread>
#include <mutex> // 用于线程同步
#include <array>
#include <termios.h>
#include <fcntl.h>

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

        // filted_joints = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        //     "filted_joints", qos
        // );
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

        // std::function<JointPosition()> rokae_callback = [&,this] ()
        // {
        //     bool is_ready_to_move = false;
        //     std::array<double,7> current_target_joint_pos_;
        //     {
        //         std::lock_guard<std::mutex> lock(joint_positions_mutex_);
        //         current_target_joint_pos_ = rec_joint_positions_;
        //         if(!init_joint_pos_set_ && has_received_joint_positions_ && init_move_completed) 
        //         {
        //             init_joint_pos_set_ = true;
        //         }
        //         is_ready_to_move = init_joint_pos_set_ ;
        //     }
        //     // else
        //     // {
        //     //     RCLCPP_INFO(this->get_logger(), "Received No init pos.Stay at zero position.");
        //     // }
        //     JointPosition cmd;
        //     // std::cout << init_joint_pos_set_<< std::endl;
        //     if(is_ready_to_move)
        //     {
        //         cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        //     }
        //     else
        //     {
        //         std::array<double, 7> current_pos {};
        //         robot_.getStateData(RtSupportedFields::jointPos_m, current_pos);
        //         cmd.joints = std::vector<double>(current_pos.begin(), current_pos.end());
        //     }
        //     return cmd;
        // };

        // motion_controller_->setControlLoop(rokae_callback);
        // motion_controller_->startMove(RtControllerMode::jointPosition);
        // control_thread_ = std::thread([this]() 
        // {
        //     this->motion_controller_->startLoop(true);
        // });

        keyboard_thread_ = std::thread([this]()
        {
            this->keyboard_input_thread();
        });
    };

    ~rt_RobotCtrlNode()
    {
        if(control_thread_.joinable())
        {
            motion_controller_->stopLoop();
            control_thread_.join();
            keyboard_thread_.join();
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
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            if(!msg->data.empty())
            {
                if(!filter_initialized_)
                {
                    for(size_t i = 0; i < 7; i++)
                    {
                        rec_joint_positions_[i] = msg->data[i];
                    }
                    filter_initialized_ = true;
                }
                else
                {
                    for(size_t i = 0; i < 7; i++)
                    {
                        rec_joint_positions_[i] = filter_alpha_ * msg->data[i] + (1 - filter_alpha_) * rec_joint_positions_[i];
                    }
                }
                has_received_joint_positions_ = true;
            }
        }
        // std_msgs::msg::Float32MultiArray filted_msg;
        // filted_msg.data.assign(rec_joint_positions_.begin(), rec_joint_positions_.end());
        // filted_joints->publish(filted_msg);
    }

    JointPosition rokae_callback()
    {
        bool is_ready_to_move = false;
        std::array<double,7> current_target_joint_pos_;
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            current_target_joint_pos_ = rec_joint_positions_;
            if(!init_joint_pos_set_ && has_received_joint_positions_ && init_move_completed) 
            {
                init_joint_pos_set_ = true;
            }
            is_ready_to_move = init_joint_pos_set_ ;
        }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Received No init pos.Stay at zero position.");
        // }
        JointPosition cmd;
        // std::cout << init_joint_pos_set_<< std::endl;
        if(is_ready_to_move)
        {
            cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        }
        else
        {
            std::array<double, 7> current_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, current_pos);
            cmd.joints = std::vector<double>(current_pos.begin(), current_pos.end());
        }
        return cmd;
    }



    int kbhit(void)
    {
        struct termios oldt, newt;
        int ch;
        int oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF)
        {
            ungetc(ch, stdin);
            return 1;
        }
        return 0;
    }

    void keyboard_input_thread()
    {
        while (rclcpp::ok())  
        {
            if (kbhit())
            {
                char ch = getchar();
                switch (ch)
                {
                    case 'm':
                    {
                        std::array<double, 7> target_pos {};
                        {
                            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                            target_pos = rec_joint_positions_;
                        }
                        if(!move_init)
                        {
                            std::array<double, 7> cur_pos {};
                            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
                            std::cout << "Current target positions: ";
                            for (double value : target_pos) {
                                std::cout << value << " ";
                            }
                            std::cout << std::endl;                
                            motion_controller_->MoveJ(0.5,cur_pos,target_pos);
                            move_init = true;
                        }
                    }
                        break;
                    case 'c':
                        if (!control_loop_started_) 
                        {
                            {
                                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                                init_move_completed = true;
                            }
                            motion_controller_->setControlLoop(
                                std::function<JointPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this))
                            );
                            motion_controller_->startMove(RtControllerMode::jointPosition);
                            RCLCPP_INFO(this->get_logger(), "Control loop started.");
                            control_thread_ = std::thread([this]() 
                            {
                                try 
                                {
                                    this->motion_controller_->startLoop(true);
                                } catch (const std::exception& e) 
                                {
                                    RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                                }
                            });
                            control_loop_started_ = true;
                        }
                        break;
                    case 'q':
                        std::cout << "Exiting..." << std::endl;
                        rclcpp::shutdown();  
                        return;
                    default:
                        break;
                }
            }
            usleep(10000);  // 10ms
        }
    }

    rokae::xMateErProRobot robot_;

    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;

    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filted_joints;
    
    const std::array<double, 7> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //线程共享数据
    std::array<double, 7> rec_joint_positions_;
    std::mutex joint_positions_mutex_;

    bool init_joint_pos_set_ = false;
    bool has_received_joint_positions_ = false;
    bool move_init = false;
    bool init_move_completed = false;
    bool control_loop_started_ = false;

    std::thread control_thread_;
    std::thread keyboard_thread_; 

    //filter variables
    const double filter_alpha_ = 0.1; //滤波系数
    bool filter_initialized_ = false;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rt_RobotCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}