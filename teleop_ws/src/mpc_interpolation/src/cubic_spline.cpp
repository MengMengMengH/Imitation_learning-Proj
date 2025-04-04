#include "../include/mpc_interpolation/cubic_spline.hpp"
#include <iostream>
#include <chrono>
#include <random>
#include <QApplication>
#include <QMainWindow>
#include <QChart>
#include <QLineSeries>

int main(int argc, char *argv[]) 
{
    QApplication app(argc, argv);
    QMainWindow window;

    QChart *chart = new QChart();
    QLineSeries *lineSeries = new QLineSeries();

    const int num_segs = 5;
    const int num_points = 50;
    const double delta = 0.15;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-delta, delta);


    Eigen::VectorXd start_pos(7);
    Eigen::VectorXd start_vel(7);
    Eigen::VectorXd start_acc(7);
    Eigen::VectorXd end_pos(7);
    start_pos << 0, 0, 0, 0, 0, 0, 0; // 起始位置
    start_vel << 0, 0, 0, 0, 0, 0, 0; // 起始速度
    start_acc << 0, 0, 0, 0, 0, 0, 0; // 起始加速度
    end_pos << 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15; // 终点位置




    for(int seg = 0;seg<num_segs; seg++)
    {   
        CubicSplineTrajectoryPlanner planner(start_pos, start_vel, start_acc, end_pos, num_points);
        try {
            auto start = std::chrono::high_resolution_clock::now();
            auto trajectory = planner.generateTrajectory();
            // auto coeffs = planner.computeCubicCoefficients();
            auto end = std::chrono::high_resolution_clock::now();
            auto solve_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            // for (const auto& point : trajectory) {
            //     std::cout << "Point: " << point.transpose() << std::endl;
            // }
            // std::cout << "Coefficients: "<< planner.coeffs << std::endl;
            std::cout << "Segment " << seg + 1 << " solve time: " << solve_time << " microseconds" << std::endl;
            if (seg < num_segs - 1) 
            {
                // Update start conditions for next segment
                start_pos = end_pos;
                start_vel = planner.evaluatePolynomial(1.0, planner.coeffs, 1);
                start_acc = planner.evaluatePolynomial(1.0, planner.coeffs, 2);
                // Randomly adjust end_pos for next segment
                for (int i = 0; i < 7; ++i) {
                    end_pos[i] += dis(gen);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
    return 0;
}