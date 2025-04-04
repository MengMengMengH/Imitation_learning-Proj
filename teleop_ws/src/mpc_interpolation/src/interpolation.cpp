#include "../include/mpc_interpolation/mpc_spline.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr int Horizon = 5;
constexpr int InputNum = 1;
constexpr int simsteps = 1;


void mpc4Axis(double dt, double src,double target,size_t index)

{
    MpcSpline<Horizon, InputNum> mpc_spline(dt, index);
    Eigen::Matrix<real_t, 4, 1> x0;
    x0 << src, 0.0, 0.0, 0.0;  // 位置、速度、加速度、jerk（初始值）

    // double total_solve_time = 0.0;
    // double total_error = 0.0;
    auto start = std::chrono::high_resolution_clock::now();
    for (int step = 0; step < simsteps; step++)
    {
        // 测量 QP 求解时间
        // auto start = std::chrono::high_resolution_clock::now();
        mpc_spline.UpdateX(x0);
        mpc_spline.UpdateXRef(target);
        // mpc_spline.UpdateXRef(target);
        Eigen::Matrix<double, 1 * Horizon, 1> u_seq = mpc_spline.RenewDeltaJerk();
        // auto end = std::chrono::high_resolution_clock::now();
        // double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // total_solve_time += solve_time;

        x0 = mpc_spline.getSimulateState();
        // double error = std::abs(x0(0) - target);
        // total_error += error;
        // std::cout << " | 当前状态：" << x0.transpose() << std::endl;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto total_solve_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "一次求解时间" << total_solve_time << " 微秒" << std::endl;
}

int main(int argc, char **argv)
{

    std::cout << "MpcSpline Func Test" << std::endl;
    double dt = 0.5;
    double init_p = 0.0;
    double target_p = 5;
    double total_solve_time = 0.0;
    double total_error = 0.0;

    // auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 7; i++)
    {
        mpc4Axis(dt,init_p,target_p,i);
    }
    // auto end = std::chrono::high_resolution_clock::now();
    // total_solve_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "\n仿真结束。" << std::endl;
    std::cout << "总求解时间" << total_solve_time << " 微秒" << std::endl;

    return 0;
}

