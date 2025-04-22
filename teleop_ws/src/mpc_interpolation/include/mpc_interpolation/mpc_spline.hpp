#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

constexpr int InputNum = 1;
constexpr int Horizon = 5;

template <int Horizon, int InputNum>
class MpcSpline
{

public:
    MpcSpline(double dt, size_t Index);
    ~MpcSpline(){delete[] xOpt_;};

    // -------------------- 状态更新 --------------------
    void setCurrentState(const Eigen::Matrix<real_t, 4, 1>& x0);
    void setReferenceTrajectory(const Eigen::Matrix<real_t, Horizon, 1>& ref_traj);


    // -------------------- 求解接口 --------------------
    bool computeMPC();

    // -------------------- 访问预测结果 --------------------
    Eigen::Matrix<real_t, Horizon, 1> getPrediction() const;

    // -------------------- 访问状态信息 --------------------
    Eigen::Matrix<real_t, 4, 1> getCurrentState() const;    
                               
    void debugDump() const;            

private:
    void UpdateConstrains();
    /* data */
    static constexpr int StateDim = 4; // pos, vel, acc, jerk 

    Eigen::Matrix<real_t, StateDim, StateDim> A_;         //A matrix for single joint
    Eigen::Matrix<real_t, StateDim, InputNum> B_;         //B matrix for single joint

    Eigen::Matrix<real_t, StateDim * Horizon, StateDim> A_phi;
    Eigen::Matrix<real_t, StateDim * Horizon, InputNum * Horizon> B_phi;
    Eigen::Matrix<real_t, StateDim * Horizon,1> x_ref_;
    Eigen::Matrix<real_t, StateDim, 1> x_0_;
    Eigen::Matrix<real_t, StateDim * Horizon, StateDim * Horizon> Q_;
    Eigen::Matrix<real_t, InputNum * Horizon, InputNum * Horizon> R_;
    Eigen::Matrix<real_t, InputNum * Horizon, InputNum * Horizon> H_;
    Eigen::Matrix<real_t, InputNum * Horizon, 1> g_;
    Eigen::Matrix<real_t, InputNum * Horizon, 1> u_;

    // Constraints: v, a
    Eigen::Matrix<real_t, 3 * InputNum * Horizon, InputNum * Horizon> C_;
    Eigen::Matrix<real_t, 3 * InputNum * Horizon, 1> lb_C;
    Eigen::Matrix<real_t, 3 * InputNum * Horizon, 1> ub_C;


    real_t vMin[7] = {-2.175, -2.175, -2.175, -2.175, -2.610, -2.610, -2.610};
    real_t vMax[7] = { 2.175,  2.175,  2.175,  2.175,  2.610,  2.610,  2.610};
    real_t aMin[7] = {-15, -7.5, -10, -10, -15, -15, -20};
    real_t aMax[7] = { 15,  7.5,  10,  10,  15,  15,  20};
    real_t jMin[7] = {-5000, -3500, -5000, -5000, -7500, -7500, -7500};
    real_t jMax[7] = { 5000,  3500,  5000,  5000,  7500,  7500,  7500};


    int nWSR_ = 10;
    size_t constrain_index;
    QProblem mpcspline_;
    Options option_mpcspline_;
    real_t *xOpt_;

    Eigen::Matrix<real_t, 4, 1> current_state_;
    Eigen::Matrix<real_t, Horizon, 1> prediction_pos_; 
    bool ref_ready_ = false;
};


