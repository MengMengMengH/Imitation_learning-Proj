#include <iostream>
#include <cmath>
#include <thread>
#include <mutex> // 用于线程同步
#include <array>
#include <termios.h>
#include <fcntl.h>
#include <deque>
#include <chrono>

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
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
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

            // motion_controller_->setCollisionBehaviour(torqueThresholds,ec);
            robot_.startReceiveRobotState(std::chrono::milliseconds(1),{RtSupportedFields::jointPos_m});

            std::array<double, 7> cur_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
            
            std::cout << "Current joint positions: ";
            for (double value : cur_pos) {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            // PowerState state = robot_.powerState(ec);
            // std::string state_str;
            // switch (state)
            // {
            //     case PowerState::on:
            //         state_str = "上电";
            //         break;
            //     case PowerState::off:
            //         state_str = "下电";
            //         break;
            //     case PowerState::unknown:
            //         state_str = "未知";
            //         break;
            //     case PowerState::estop:
            //         state_str = "急停";
            //         break;
            //     case PowerState::gstop:
            //         state_str = "安全门打开";
            //         break;
            //     default:
            //         state_str = "无效状态";
            //         break;
            // }
            // std::cout << "Robot power state: " << state_str << std::endl;
            
            
            motion_controller_->MoveJ(0.5,robot_.jointPos(ec),zero_pos);
            RCLCPP_INFO(this->get_logger(), "Robot joint positions initialized to zero.");

            // std::error_code ec;

            // std::array<double,7> joint_torque_measured,external_torque_measured;
            // std::array<double, 3> cart_torque,cart_force;
            // robot_.getEndTorque(FrameType::world,joint_torque_measured,external_torque_measured,cart_torque,cart_force,ec);
            
            // std::cout << "Current ex_tq : ";
            // for (double value : external_torque_measured) {
            //     std::cout << value << " ";
            // }
        }
        catch(std::exception &e)
        {
            std::cerr << e.what();
        }



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
            if (keyboard_thread_.joinable())
                keyboard_thread_.join();
        }
        std::error_code ec;
        robot_.setPowerState(false, ec);
        std::cout<< "Robot power off." << std::endl;
    }

private:

    void jointPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() == 7)
        {
            std::array<double, 7> new_positions;
            for (size_t i = 0; i < 7; i++)
            {
                new_positions[i] = static_cast<double>(msg->data[i]);
            }
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                if(joint_queue_.size() >= max_queue_size_) {
                    joint_queue_.pop_front();
                }
                joint_queue_.push_back(new_positions);
            }
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
    }

    

    JointPosition rokae_callback()
    {
        std::error_code ec;
        bool is_ready_to_move = false;
        bool has_new_command = false;
        std::array<double,7> current_target_joint_pos_ {};
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            if(!joint_queue_.empty()) 
            {
                current_target_joint_pos_ = joint_queue_.front();
                joint_queue_.pop_front();
                last_valid_command_ = current_target_joint_pos_;
                has_new_command = true;
            }
            if(!init_joint_pos_set_ && init_move_completed)
            {
                init_joint_pos_set_ = true;
            }
            is_ready_to_move = init_joint_pos_set_ ;
        }

        JointPosition cmd;
        // std::cout << init_joint_pos_set_<< std::endl;
        if(is_ready_to_move && has_new_command)
        {
            cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        }
        else if(is_ready_to_move &&  !has_new_command)
        {
            cmd.joints = std::vector<double>(last_valid_command_.begin(), last_valid_command_.end());
        }
        else
        {
            std::array<double,7> cur_jntPos = robot_.jointPos(ec);
            cmd.joints = std::vector<double>(cur_jntPos.begin(),cur_jntPos.end());
        }
        // for (double value : cmd.joints) {
        //         std::cout << value << " ";
        //     }
        //     std::cout << std::endl;
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
                    case 'c':
                        followed_position_loop();
                        break;
                    case 'v':
                        followed_impedance_loop();
                        break;
                    case 'q':
                        std::cout << "Exiting..." << std::endl;
                        rclcpp::shutdown();  
                        return;
                    default:
                        break;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    void followed_impedance_loop()
    {
        if (!control_loop_started_) 
        {
            std::error_code ec;
            std::array<double, 7> target_pos {};
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                target_pos = joint_queue_.empty() ? zero_pos : joint_queue_.back();
                joint_queue_.clear();
            }

            std::cout << "Current target positions: ";
            for (double value : target_pos) {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            if (!motion_controller_)
            {
                RCLCPP_FATAL(this->get_logger(),"Motion Controller error!");
                rclcpp::shutdown();
                return;
            }

            size_t que_size = 0;
            do
            {
                motion_controller_->MoveJ(0.3,robot_.jointPos(ec),target_pos);
                {
                    std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                    que_size = joint_queue_.size();
                    target_pos = joint_queue_.back();
                    joint_queue_.clear();
                }
                std::cout<< que_size << std::endl;
            } while(que_size < 25);


            motion_controller_->setJointImpedance({300, 300, 300, 500, 50, 100, 50}, ec);
            motion_controller_->startMove(RtControllerMode::jointImpedance);
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                init_move_completed = true;
            }
            motion_controller_->setControlLoop(
                std::function<JointPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)),
                0,
                true
            );
            RCLCPP_INFO(this->get_logger(), "Control loop started.");
            control_thread_ = std::thread([this]() 
            {
                try 
                {
                    // t0 = std::chrono::high_resolution_clock::now();
                    this->motion_controller_->startLoop(true);
                } catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                }
            });
            control_loop_started_ = true;
        }
    }

    void followed_position_loop()
    {
        if (!control_loop_started_) 
        {
            std::error_code ec;
            std::array<double, 7> target_pos {};
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                target_pos = joint_queue_.empty() ? zero_pos : joint_queue_.back();
                joint_queue_.clear();
            }
            robot_.updateRobotState(5ms);

            // std::this_thread::sleep_for(20ms);

            std::cout << "Current target positions: ";
            for (double value : target_pos) {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            if (!motion_controller_)
            {
                RCLCPP_FATAL(this->get_logger(),"Motion Controller error!");
                rclcpp::shutdown();
                return;
            }

            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                size_t que_size = joint_queue_.size();
                std::cout <<"MoveJ before:"<< que_size << std::endl;
            }

            motion_controller_->MoveJ(0.3,robot_.jointPos(ec),target_pos);

            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                size_t que_size = joint_queue_.size();
                std::cout <<"MoveJ after:"<< que_size << std::endl;
            }

            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                init_move_completed = true;
            }
            motion_controller_->setControlLoop(
                std::function<JointPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)),
                0,
                true
            );
            motion_controller_->startMove(RtControllerMode::jointPosition);
            RCLCPP_INFO(this->get_logger(), "Control loop started.");

            control_thread_ = std::thread([this]()
            {
                try 
                {
                    // t0 = std::chrono::high_resolution_clock::now();
                    this->motion_controller_->startLoop(true);
                } catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                }
            });
            control_loop_started_ = true;
        }
    }


    rokae::xMateErProRobot robot_;

    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;

    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filted_joints;
    
    const std::array<double, 7> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::array<double, 7> debug_pos = {0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0};


    const std::array<double, 7> torqueThresholds = { 75, 75, 60, 45, 30, 30, 20 };
    std::array<double, 7> last_valid_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //线程共享数据
    std::mutex joint_positions_mutex_;
    std::deque<std::array<double,7>> joint_queue_;
    const size_t max_queue_size_ = 30; // 定长队列大小


    bool init_joint_pos_set_ = false;
    bool init_move_completed = false;
    bool control_loop_started_ = false;

    std::thread control_thread_;
    std::thread keyboard_thread_; 
    std::chrono::time_point<std::chrono::high_resolution_clock> t0;
    int count = 0;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rt_RobotCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}