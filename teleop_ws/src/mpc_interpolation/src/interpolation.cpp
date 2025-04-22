#include <iostream>
#include <chrono>
#include <queue>
#include <array>
#include <sstream> 
#include <mutex>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "../include/mpc_interpolation/mpc_spline.hpp"
#include "../include/mpc_interpolation/cubic_spline.hpp"


constexpr int jointSize = 7;


class interpolation : public rclcpp::Node
{
public:

    explicit interpolation() : Node("interpolation_node")
    {
        // Initialize the node
        RCLCPP_INFO(this->get_logger(), "Interpolation node started.");

        roake_control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/roake_control_joints", 10, std::bind(&interpolation::exec_planning_callback, this,std::placeholders::_1));
        
        running_ = true;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            mpc_traj_.push(Eigen::VectorXd::Zero(jointSize));
        }
        mpc_thread_ = std::thread(&interpolation::mpcTrajCompute, this);

        mimic_send_goal_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "mimic_send_joint", 10);
        mimic_thread_ = std::thread(&interpolation::mimicSendGoal, this);
        
    }

    ~interpolation()
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            running_ = false;
        }
        if (mpc_thread_.joinable())
        {
            mpc_thread_.join();
        }
        if (mimic_thread_.joinable())
        {
            mimic_thread_.join();
        }
    }

private:


    void exec_planning_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        end_position = conv2EigenVec(msg->data);
        getRefPoints(start_position, start_velocity, end_position, IPT_NUM);
    }

    void mimicSendGoal()
    {
        rclcpp::Rate rate(1000);
        Eigen::Vector<real_t, jointSize> last_valid_goal = Eigen::Vector<real_t, jointSize>::Zero();
        static int empty_counter = 0;
        while(rclcpp::ok() && running_)
        {
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                if (!mpc_traj_.empty())
                {
                    last_valid_goal = mpc_traj_.front();
                    mpc_traj_.pop();
                }
            }    
            std_msgs::msg::Float32MultiArray msg;
            msg.data.resize(jointSize);
            for (size_t i = 0; i < jointSize; i++)
            {
                msg.data[i] = last_valid_goal(i);
            }
            mimic_send_goal_->publish(msg);
            if (mpc_traj_.empty())
            {
                if (++empty_counter % 1000 == 0) 
                {
                    std::stringstream ss = lst2stream(std::vector<float>(last_valid_goal.data(), last_valid_goal.data() + jointSize));
                    RCLCPP_WARN(this->get_logger(), "mpc_traj_ is empty, sending last known value: \n%s", ss.str().c_str());
                }
            }
            else { empty_counter = 0; }
            rate.sleep();
        }
    }

    /**
     * @brief Computes MPC trajectory using a receding horizon approach.
     * Waits for sufficient reference points, calls getNextMpcPos, and manages the reference window.
     */
    void mpcTrajCompute()
    {
        rclcpp::Rate rate(1000);
        while(rclcpp::ok() && running_)
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait_for(lock, std::chrono::milliseconds(5), [&]() {
                return !running_ ||
                       (mpc_ref.size() < Horizon && des_traj_.size() >= (Horizon - mpc_ref.size())) || // 有足够的点来填充初始窗口
                       (mpc_ref.size() == Horizon && des_traj_.size() >= N_apply);                   // 有足够的点来滑动窗口
            });
            if (!running_){break;}


            while (mpc_ref.size() < Horizon && !des_traj_.empty())
            {
                mpc_ref.push(des_traj_.front());
                des_traj_.pop();
            }
            if (mpc_ref.size() < Horizon || des_traj_.size() < N_apply)
            {
                lock.unlock();
                rate.sleep(); 
                continue;
            }
            lock.unlock();

            assert(mpc_ref.size() == Horizon && "mpc ref size error");

            bool success = getNextMpcPos();

            if(success)
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                for (size_t i = 0; i < N_apply; ++i)
                {
                    if (!mpc_ref.empty())
                    {
                        mpc_ref.pop(); // 移除最旧的参考点
                    }
                    if (!des_traj_.empty())
                    {
                        mpc_ref.push(des_traj_.front()); // 添加最新的参考点
                        des_traj_.pop();
                    }
                    else
                    {
                        // 由于等待条件，这不应发生，但做防御性处理
                        RCLCPP_WARN(this->get_logger(), "滑动窗口时 des_traj_ 意外为空!");
                        // 选项：复制 mpc_ref 中最后一个点以保持大小？
                        if (!mpc_ref.empty()) {
                           mpc_ref.push(mpc_ref.back());
                        }
                    }
                }
                assert(mpc_ref.size() == Horizon && "滑动后 MPC 参考窗口大小不正确");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "MPC 计算失败!");
                // TODO:
                //--- 需要错误处理策略 ---
            }
            rate.sleep();
        }
    }

    /**
     * @brief 使用当前参考窗口执行一步 MPC 计算
     * 使用上次计算的状态（首次运行时使用参考）作为初始状态
     * 将计算出的第一个 N_apply 点推送到 mpc_traj_
     * @return 如果所有关节计算成功则返回 true，否则 false
     */

    bool getNextMpcPos()
    {
        Eigen::Matrix<real_t, jointSize, Horizon> ref_pos_horizon; // 存储从队列复制的参考轨迹
        Eigen::Matrix<real_t, jointSize, 1> current_x0_pos;
        // TODO: 
        // 如果需要/可用，添加速度/加速度/加加速度状态变量

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (mpc_ref.size() < Horizon) 
            {
                RCLCPP_ERROR(this->get_logger(), "调用 MPC 计算时 参考点 大小 %zu < 预测步长 %d", mpc_ref.size(), Horizon);
                return false; // 无法计算
            }
            std::queue<Eigen::VectorXd> temp_ref_copy = mpc_ref; 
            for (int i = 0; i < Horizon; ++i) 
            {
                ref_pos_horizon.col(i) = temp_ref_copy.front();
                // std::cout << "ref_pos_horizon.col(" << i << "): " << ref_pos_horizon.col(i).transpose() << std::endl;
                temp_ref_copy.pop();
            }

            //确定 MPC 的初始状态 x0
            if (first_mpc_run_)
            {
                // 首次运行，使用第一个参考点作为位置
                current_x0_pos = ref_pos_horizon.col(0);
                // TODO: 理想情况下，即使在这里也应尽可能从实际机器人状态获取初始速度/加速度
                first_mpc_run_ = false; // 清除标志，下次不再使用此逻辑直到重置
                    RCLCPP_INFO(this->get_logger(), "首次 MPC 运行，从参考初始化 x0。");
            } 
            else 
            {
                // 对于后续运行，使用上一步计算出的位置
                current_x0_pos = last_computed_pos_;
                // TODO: 从机器人状态反馈或上次 MPC 预测中获取速度/加速度
            }
        } // 锁定范围结束

        Eigen::Matrix<real_t, jointSize, Horizon> MPCpos_horizon; // 存储计算出的完整时域轨迹
        bool computation_ok = true; // 标记计算是否对所有关节都成功
        for(int i = 0; i < jointSize; i++)
        {
            Eigen::Matrix<real_t,4,1> x0;
            x0 << current_x0_pos(i), 0, 0, 0; 
            // TODO:
            /* using robot real state to update x0 */
            MPCsplines_[i].setCurrentState(x0);
            MPCsplines_[i].setReferenceTrajectory(ref_pos_horizon.row(i));
            if(MPCsplines_[i].computeMPC())
            {
                auto x_pred = MPCsplines_[i].getPrediction();
                MPCpos_horizon.row(i) = x_pred.transpose();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "MPC 计算失败，关节 %d", i);
                computation_ok = false;
                break; // 如果任何关节失败，则退出
            }
        }

        // MPCsplines_[0].debugDump();

        if(computation_ok)
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            for(size_t i = 0; i < N_apply; ++i) {
                mpc_traj_.push(MPCpos_horizon.col(i)); // 推送计算结果的第一步（或前 N_apply 步）
            }
            // 存储*最后一个应用点*的位置，用于下一次迭代的 x0
            last_computed_pos_ = MPCpos_horizon.col(N_apply - 1);
            // TODO: 如果需要用于 x0，也在此处存储上次计算的速度等

            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "一个或多个关节的 MPC 步骤失败。");
            // 可选地清除上次计算的状态？
            first_mpc_run_ = true; // 强制重新初始化？
            return false; // 表示失败
        }
    }


    void getRefPoints(Eigen::VectorXd start_pos, Eigen::VectorXd start_vel,Eigen::VectorXd end_pos,int num_points)
    {
        CubicSplineTrajectoryPlanner planner(start_pos, start_vel, end_pos, num_points);
        planner.coeffs = planner.computeCubicCoefficients();

        std::unique_lock<std::mutex> lock(queue_mutex_);
        for (int i = 0; i < num_points; i++)
        {
            double t = static_cast<double>(i) / (num_points - 1);
            Eigen::VectorXd point = planner.evaluatePolynomial(t, planner.coeffs);
            des_traj_.push(point);
        }
        lock.unlock();
        queue_cv_.notify_one();
        //update start state
        start_position = end_pos;
        start_velocity = planner.evaluatePolynomial(1.0, planner.coeffs, 1);
    }



    std::stringstream lst2stream(const std::vector<float>& lst)
    {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < lst.size(); ++i) 
        {
            ss << lst[i];
            if (i != lst.size() - 1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss;
    }

    Eigen::VectorXd conv2EigenVec(const std::vector<float>& lst)
    {
        std::vector<double> double_angles(lst.begin(), lst.end());
        Eigen::Map<const Eigen::VectorXd> vec(double_angles.data(), double_angles.size());
        return vec;
    }

    const int IPT_NUM = 20;


    Eigen::VectorXd start_position = Eigen::VectorXd::Zero(jointSize);
    Eigen::VectorXd end_position = Eigen::VectorXd::Zero(jointSize);
    Eigen::VectorXd start_velocity = Eigen::VectorXd::Zero(jointSize);


    std::queue<Eigen::VectorXd> des_traj_;
    std::queue<Eigen::VectorXd> mpc_traj_;
    std::queue<Eigen::VectorXd> mpc_ref;

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread mpc_thread_;
    std::thread mimic_thread_;
    bool running_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr roake_control_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mimic_send_goal_;


    std::array<MpcSpline<Horizon, InputNum>, 7> MPCsplines_= []<size_t... I>(std::index_sequence<I...>) {
        return std::array{MpcSpline<Horizon, InputNum>(0.5, I)...};
    }(std::make_index_sequence<7>{});

    const unsigned int N_apply = 1;
    Eigen::Matrix<real_t, jointSize, 1> last_computed_pos_ = Eigen::VectorXd::Zero(jointSize);
    bool first_mpc_run_ = true;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interpolation>();
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}