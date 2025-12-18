#include <iostream>
#include <chrono>
#include <array>
#include <sstream> 
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <queue>

#include <boost/lockfree/queue.hpp>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "../include/mpc_interpolation/mpc_spline.hpp"
#include "../include/mpc_interpolation/cubic_spline.hpp"


constexpr int jointSize = 7;
constexpr double mpc_dt = 0.001; // 1000Hz
constexpr uint32_t MPC_START_REF = 40;
constexpr uint32_t MIMIC_START_MPC_NUM = 10;
// constexpr size_t MPC_BUFFER_CAP = 16;
using Vector7d = Eigen::Vector<real_t, jointSize>;
using Matrix7H = Eigen::Matrix<real_t, jointSize, Horizon>;
using JointArray = std::array<double, jointSize>;

using namespace std::chrono_literals;

bool setRealtimePriority(pthread_t thread, int priority)
{
    sched_param sch_params;
    sch_params.sched_priority = priority;

    if (pthread_setschedparam(thread, SCHED_FIFO, &sch_params)) 
    {
        std::cerr << "Failed to set thread priority (need CAP_SYS_NICE)." << std::endl;
        return false;
    }
    return true;
}

bool setThreadAffinity(pthread_t thread, int cpu_core_id)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core_id, &cpuset);

    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) 
    {
        std::cerr << "Failed to set thread affinity to CPU " << cpu_core_id << std::endl;
        return false;
    }
    return true;
}

class interpolation : public rclcpp::Node
{
public:

    explicit interpolation() : Node("interpolation_node"),running_(true)
    {
        // Initialize the node
        RCLCPP_INFO(this->get_logger(), "Interpolation node started.");

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile)
            .deadline(rclcpp::Duration(1ms));

        roake_control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rokae_control_joints", qos, std::bind(&interpolation::update_ref_points, this,std::placeholders::_1));
        
        // {
        //     std::lock_guard<std::mutex> lock(queue_mutex_);
        //     mpc_traj_.push(Eigen::VectorXd::Zero(jointSize));
        // }
        exec_plan_thread_ = std::thread(&interpolation::exec_planning,this);
        pthread_t handle_exec = exec_plan_thread_.native_handle();
        setRealtimePriority(handle_exec,85);
        setThreadAffinity(handle_exec,1);

        mpc_thread_ = std::thread(&interpolation::mpcTrajCompute, this);
        //  设置 mpc_thread_ 为实时线程 + 绑定 CPU core1
        pthread_t handle_mpc = mpc_thread_.native_handle();
        setRealtimePriority(handle_mpc,85);
        setThreadAffinity(handle_mpc,2);

        mimic_send_goal_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos);
        mimic_thread_ = std::thread(&interpolation::mimicSendGoal, this);

        //  设置 mimic_thread_ 为实时线程 + 绑定 CPU core2
        pthread_t handle = mimic_thread_.native_handle();
        setRealtimePriority(handle, 75);  // 建议 40~70 之间，不要太高
        setThreadAffinity(handle, 3);     // 绑到 CPU core2
    }

    ~interpolation()
    {
        running_ = false;
        if (mpc_thread_.joinable()) mpc_thread_.join();
        if (mimic_thread_.joinable())mimic_thread_.join();
    }

private:


    void update_ref_points(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        Vector7d new_target = conv2EigenVec(msg->data);
        {
            std::lock_guard<std::mutex> lk(target_mtx_);
            end_position_ = new_target;
            target_updated_ = true;
        }
    }

    void exec_planning()
    {
        rclcpp::Rate rate(50);

        Vector7d local_end_position;
        while(rclcpp::ok() && running_)
        {
            {
                std::lock_guard<std::mutex> lk(target_mtx_);
                if (target_updated_)
                {
                    local_end_position = end_position_;
                    target_updated_ = false;
                }
            }

            CubicSplineTrajectoryPlanner planner(
                start_position_, start_velocity_, local_end_position, IPT_NUM);
            planner.coeffs = planner.computeCubicCoefficients();
            for (int i = 0; i < IPT_NUM; ++i)
            {
                double t = static_cast<double>(i) / (IPT_NUM - 1);
                JointArray pt{};
                Vector7d eigen_pt = planner.evaluatePolynomial(t, planner.coeffs);
                for (int j = 0; j < jointSize; ++j)
                {
                    pt[j] = eigen_pt(j);
                }
                if (ref_traj_queue_.push(pt)) 
                {
                    ref_produced_.fetch_add(1, std::memory_order_relaxed);
                }
            }

            if (!mpc_enabled_.load(std::memory_order_acquire) &&
                ref_produced_.load(std::memory_order_relaxed) >= MPC_START_REF)
            {
                mpc_enabled_.store(true, std::memory_order_release);
            }

            start_position_ = local_end_position;
            start_velocity_ = planner.evaluatePolynomial(1.0, planner.coeffs, 1);

            rate.sleep();
        }
    }

    void mimicSendGoal()
    {
        rclcpp::Rate rate(1000);

        while (rclcpp::ok() && running_)
        {
            if (!mimic_enabled_.load(std::memory_order_acquire)) 
            {
                rate.sleep();
                continue;
            }

            JointArray cmd;
            if (!mpc_traj_queue_.pop(cmd)) 
            {
                // 这里理论上不应该发生
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "mimic enabled but mpc_traj_queue empty!"
                );
                rate.sleep();
                continue;
            }

            std_msgs::msg::Float32MultiArray msg;
            msg.data.resize(jointSize);
            for (int i = 0; i < jointSize; ++i)
                msg.data[i] = static_cast<float>(cmd[i]);
            mimic_send_goal_->publish(msg);
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
        std::deque<JointArray> ref_window;

        while(rclcpp::ok() && running_)
        {

            // uint64_t prod = ref_produced_.load();
            // uint64_t cons = ref_consumed_.load();

            if (!mpc_enabled_.load(std::memory_order_acquire)) 
            {
                // printf("[REF STAT] produced=%lu consumed=%lu\n",
                // prod, cons);
                rate.sleep();
                continue;
            }
            // printf("[REF STAT] produced=%lu consumed=%lu in_queue≈%ld\n",
            //     prod, cons, (long)(prod - cons));
            auto t0 = std::chrono::high_resolution_clock::now();            
            JointArray raw{};
            Vector7d pt;
            while (ref_window.size() < Horizon && ref_traj_queue_.pop(raw))
            {
                ref_window.push_back(raw);
                ref_consumed_.fetch_add(1, std::memory_order_relaxed);
            }

            if (ref_window.size() < Horizon) 
            {
                printf("ref windows less than 5\n");
                rate.sleep();
                continue;
            }

            Matrix7H ref_horizon;
            for (int i = 0; i < Horizon; ++i)
            {
                for (int j = 0; j < jointSize; ++j)
                {
                    ref_horizon(j, i) = ref_window[i][j];
                }
            }

            Vector7d x0 = first_mpc_run_ ? ref_horizon.col(0) : last_pos_;
            if (first_mpc_run_) {
                last_vel_.setZero();
                last_acc_.setZero();
                first_mpc_run_ = false;
            }
            
            Matrix7H mpc_out;
            bool mpc_compute_ok = true;
            for (int j = 0; j < jointSize; ++j)
            {
                Eigen::Vector3d state0;
                state0 << x0(j), last_vel_(j), last_acc_(j);

                MPCsplines_[j].setCurrentState(state0);
                MPCsplines_[j].setReferenceTrajectory(ref_horizon.row(j));

                if (!MPCsplines_[j].computeMPC()) {
                    mpc_compute_ok = false;
                    break;
                }
                mpc_out.row(j) = MPCsplines_[j].getPrediction().transpose();
                auto full = MPCsplines_[j].getFullStatePrediction();
                last_vel_(j) = full(1);
                last_acc_(j) = full(2);
            }
            if (mpc_compute_ok)
            {
                JointArray mpc_out_arr{};
                for(int j = 0;j<jointSize;j++)
                {
                    mpc_out_arr[j] = mpc_out(j,N_apply - 1);
                }

                if(mpc_traj_queue_.push(mpc_out_arr))
                {
                    mpc_produced_.fetch_add(1, std::memory_order_relaxed);
                }
                if (!mimic_enabled_.load(std::memory_order_acquire) &&
                    mpc_produced_.load(std::memory_order_relaxed) >= MIMIC_START_MPC_NUM)
                {
                    mimic_enabled_.store(true, std::memory_order_release);
                }

                last_pos_ = mpc_out.col(N_apply - 1);
                for (unsigned i = 0; i < N_apply; ++i)
                {    
                    ref_window.pop_front();
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),"MPC Compute Failed!");
            }

            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::high_resolution_clock::now() - t0)
                          .count();

            if (dt > 1200)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "MPC loop took %ld us", dt);

            rate.sleep();
        }
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

    Vector7d conv2EigenVec(const std::vector<float>& lst)
    {
        Vector7d out;
        for (size_t i = 0; i < lst.size(); ++i) out(i)= lst[i];
        return out;
    }

    const int IPT_NUM = 20;
    // ===================== 无锁队列 =====================
    // 上层规划 → MPC
    boost::lockfree::queue<JointArray, boost::lockfree::capacity<8192>> ref_traj_queue_;
    // MPC → 发送线程
    boost::lockfree::queue<JointArray, boost::lockfree::capacity<4096>> mpc_traj_queue_;
    // boost::lockfree::queue<JointArray,boost::lockfree::capacity<MPC_BUFFER_CAP>> mpc_buffer_queue_;

    // ===================== 状态 =====================
    Vector7d start_position_ = Vector7d::Zero();
    Vector7d start_velocity_ = Vector7d::Zero();
    Vector7d end_position_ = Vector7d::Zero();
    std::mutex target_mtx_;
    bool target_updated_{false};

    Vector7d last_pos_ = Vector7d::Zero();
    Vector7d last_vel_ = Vector7d::Zero();
    Vector7d last_acc_ = Vector7d::Zero();
    bool first_mpc_run_ = true;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr roake_control_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mimic_send_goal_;
    
    std::array<MpcSpline<Horizon, InputNum>, 7> MPCsplines_= []<size_t... I>(std::index_sequence<I...>) {
        return std::array{MpcSpline<Horizon, InputNum>(mpc_dt, I)...};
    }(std::make_index_sequence<7>{});

    std::atomic<bool> running_;
    std::thread mpc_thread_, mimic_thread_,exec_plan_thread_;

    std::atomic<bool> mpc_enabled_{false};
    std::atomic<uint32_t> ref_produced_{0};
    std::atomic<uint64_t> ref_consumed_{0};

    std::atomic<uint32_t> mpc_produced_{0};    // MPC 已产生的点数
    std::atomic<bool> mimic_enabled_{false};  // mimic 启动闸门

    const unsigned int N_apply = 1;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interpolation>();
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}