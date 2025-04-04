#pragma once
#include <vector>
#include <Eigen/Dense>
#include <iostream>

class CubicSplineTrajectoryPlanner {
public:
    // 构造函数，传入起始点位置、速度、加速度及终点位置
    CubicSplineTrajectoryPlanner(const Eigen::VectorXd& start_pos,
                                 const Eigen::VectorXd& start_vel,
                                 const Eigen::VectorXd& start_acc,
                                 const Eigen::VectorXd& end_pos,
                                 int num_points)
        : p0(start_pos), v0(start_vel), a0(start_acc), p1(end_pos), N(num_points) {
        if (p0.size() != p1.size() || v0.size() != p0.size() || a0.size() != p0.size()) {
            throw std::invalid_argument("All input vectors must have the same dimension.");
        }
    }
    Eigen::MatrixXd coeffs;

    // 生成轨迹
    std::vector<Eigen::VectorXd> generateTrajectory() {
        std::vector<Eigen::VectorXd> trajectory;
        coeffs = computeCubicCoefficients();

        for (int i = 0; i < N; ++i) {
            double t = static_cast<double>(i) / (N - 1);
            Eigen::VectorXd point = evaluatePolynomial(t, coeffs);
            trajectory.push_back(point);
        }
        return trajectory;
    }

        // 计算三次样条系数
    Eigen::MatrixXd computeCubicCoefficients() {
        int dim = p0.size();
        Eigen::MatrixXd coeffs(4, dim);
        for (int d = 0; d < dim; ++d) {
            Eigen::Matrix4d A;
            Eigen::Vector4d b;
            // 设置边界条件矩阵
            A << 1, 0, 0, 0,    // p(0) = p0
                    0, 1, 0, 0,    // p'(0) = v0
                    0, 0, 2, 0,    // p''(0) = a0
                    1, 1, 1, 1;    // p(1) = p1

            b << p0[d], v0[d], a0[d], p1[d];

            // 解线性方程组 Ax = b
            coeffs.col(d) = A.colPivHouseholderQr().solve(b);
        }
        return coeffs;
    }

    /* 
        计算多项式值及其导数
    */
    Eigen::VectorXd evaluatePolynomial(double t, const Eigen::MatrixXd& coeffs, int derivative = 0) 
    {
        int dim = coeffs.cols();
        Eigen::VectorXd result(dim);
        for (int d = 0; d < dim; d++) {
            double val = 0;
            if (derivative == 0) { // 位置
                for (int i = 0; i < 4; i++) val += coeffs(i, d) * std::pow(t, i);
            } else if (derivative == 1) { // 速度
                for (int i = 1; i < 4; i++) val += i * coeffs(i, d) * std::pow(t, i - 1);
            } else if (derivative == 2) { // 加速度
                for (int i = 2; i < 4; i++) val += i * (i - 1) * coeffs(i, d) * std::pow(t, i - 2);
            } else if (derivative == 3) { // 加加速度
                for (int i = 3; i < 4; i++) val += i * (i - 1) * (i - 2) * coeffs(i, d) * std::pow(t, i - 3);
            }
            result[d] = val;
        }
        return result;
    }
    
private:

    // 成员变量
    Eigen::VectorXd p0, v0, a0; // 起始点位置、速度、加速度
    Eigen::VectorXd p1;         // 终点位置
    int N;                      // 采样点数
};