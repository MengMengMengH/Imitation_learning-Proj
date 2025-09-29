#include "../include/mpc_interpolation/mpc_spline.hpp"

template <int Horizon, int InputNum>
MpcSpline<Horizon,InputNum>::MpcSpline(double dt, size_t Index) :
    mpcspline_(InputNum * Horizon, 3 * Horizon),
    xOpt_(new real_t[InputNum * Horizon])
{
    assert(Index < 7 && "This implementation supports for 7 joints only");
    constrain_index = Index;
    //define A and B matrix
    A_ << 1.0, dt,  dt * dt / 2.0,
          0.0, 1.0, dt,
          0.0, 0.0, 1.0,

    
    B_ << dt * dt / 2.0,
          dt,
          1.0;
    // initialize A_phi and B_phi 
    A_phi.setZero();
    B_phi.setZero();
    A_phi.block(0,0,StateDim,StateDim) = A_;
    B_phi.block(0,0,StateDim,InputNum) = B_;

    for (int i = 1; i < Horizon; i++)
    {
        A_phi.block(i * StateDim, 0, StateDim, StateDim) = A_phi.block((i - 1) * StateDim, 0, StateDim, StateDim) * A_;
        for (int j = 0; j < i; j++)
        {
            B_phi.block(i * StateDim, j * InputNum, StateDim, InputNum) = A_phi.block((i - j - 1) * StateDim, 0, StateDim, StateDim) * B_;
        }
        B_phi.block(i * StateDim, i * InputNum, StateDim, InputNum) = B_;
    }

    // initialize x_0 and x_ref
    x_0_.setZero();
    x_ref_.setZero();

    // define weight matrix Q and R
    Q_.setZero();
    R_.setZero();
    for(int i = 0; i < Horizon; i++)
    {
        Q_.block(i * StateDim, i * StateDim, StateDim, StateDim).diagonal() << 1.0e5, 1.0e2, 1.0;
        R_(i, i) = 1.0e-4;
    }
    // std::cout<<"Q_ = \n" << Q_ << std::endl;
    // compute H and g
    H_ = (B_phi.transpose() * Q_ * B_phi + R_);
    g_ = B_phi.transpose() * Q_ * (A_phi * x_0_ - x_ref_);


    ////      Debug
    // double minB = B_.minCoeff();
    // double maxB = B_.maxCoeff();
    // std::cerr << "[DEBUG] B_ min=" << minB << " max=" << maxB << " norm=" << B_.norm() << std::endl;

    // double minBphi = B_phi.minCoeff();
    // double maxBphi = B_phi.maxCoeff();
    // std::cerr << "[DEBUG] B_phi min=" << minBphi << " max=" << maxBphi << " norm=" << B_phi.norm() << std::endl;

    // QP constraints
    C_.setZero();
    lb_C.setZero();
    ub_C.setZero();
    // std::cout << "B_phi = \n" << B_phi << std::endl;
    for (int i = 0; i < Horizon; i++)
    {
        // velocity constraints
        C_.block(i, 0, 1, InputNum * Horizon) = B_phi.block(i * StateDim + 1, 0, 1, InputNum * Horizon);
        // acceleration constraints
        C_.block(Horizon + i, 0, 1, InputNum * Horizon) = B_phi.block(i * StateDim + 2, 0, 1, InputNum * Horizon);
    }
        // direct u rows
    int base = 2 * Horizon;
    for (int i = 0; i < Horizon; ++i) {
        C_(base + i, i) = 1.0; // pick u_i
    }
        // delta-u rows (jerk ~ (u_{k+1}-u_k)/dt)
    base += Horizon;
    for (int k = 0; k < Horizon - 1; ++k) {
        C_(base + k, k) = -1.0 / (dt);
        C_(base + k, k+1) = 1.0 / (dt);
    }


    // initialize options
    option_mpcspline_.printLevel = PL_NONE;
    option_mpcspline_.terminationTolerance = 1e-6;
    mpcspline_.setOptions( option_mpcspline_ );
    nWSR_ = 100;

    // initialize QP solver
    returnValue status = mpcspline_.init(
        H_.data(),
        g_.data(),
        C_.data(),
        nullptr,
        nullptr,
        lb_C.data(),
        ub_C.data(),
        nWSR_
    );
    if (status != SUCCESSFUL_RETURN) 
    {
        printf("init failed with return value %d\n", status);
    }
    mpcspline_.getPrimalSolution(xOpt_);

}

template<int Horizon, int InputNum>
void MpcSpline<Horizon, InputNum>::setCurrentState(const Eigen::Matrix<real_t, 3, 1>& x0) 
{
    x_0_ = x0;
    current_state_ = x0;
}

template<int Horizon, int InputNum>
Eigen::Matrix<real_t, 3 * Horizon, 1> MpcSpline<Horizon, InputNum>::getFullStatePrediction() const {
    Eigen::Map<const Eigen::Matrix<real_t, InputNum * Horizon, 1>> xOpt_vec(xOpt_);
    return A_phi * x_0_ + B_phi * xOpt_vec;
}

template<int Horizon, int InputNum>
void MpcSpline<Horizon, InputNum>::setReferenceTrajectory(const Eigen::Matrix<real_t, Horizon, 1>& ref_traj) 
{
    for (int i = 0; i < Horizon; i++) {
        x_ref_.block(i * StateDim, 0, 1, 1) << ref_traj(i);
    }
    ref_ready_ = true;
}


template<int Horizon, int InputNum>
void MpcSpline<Horizon, InputNum>::UpdateConstrains() 
{
    lb_C.setZero();
    ub_C.setZero();
    for (int i = 0; i < Horizon; i++)
    {
        // velocity constraints
        lb_C(i) = vMin[constrain_index] - (A_phi.block(i * StateDim + 1, 0, 1, StateDim) * x_0_)(0);
        ub_C(i) = vMax[constrain_index] - (A_phi.block(i * StateDim + 1, 0, 1, StateDim) * x_0_)(0);
        // acceleration constraints
        lb_C(Horizon + i) = aMin[constrain_index] - (A_phi.block(i * StateDim + 2, 0, 1, StateDim) * x_0_)(0);
        ub_C(Horizon + i) = aMax[constrain_index] - (A_phi.block(i * StateDim + 2, 0, 1, StateDim) * x_0_)(0);
    }
    // direct u bounds (u is acceleration)
    int base = 2 * Horizon;
    for (int i = 0; i < Horizon; i++) 
    {
        lb_C(base + i) = aMin[constrain_index];
        ub_C(base + i) = aMax[constrain_index];
    }
    // delta-u (jerk) bounds: (u_{k+1}-u_k)/dt ∈ [jMin, jMax]
    base += Horizon;
    for (int k = 0; k < Horizon - 1; k++) 
    {
        lb_C(base + k) = jMin[constrain_index];
        ub_C(base + k) = jMax[constrain_index];
    }
}

template<int Horizon, int InputNum>
bool MpcSpline<Horizon, InputNum>::computeMPC() 
{
    if (!ref_ready_) {
        std::cerr << "[MPC] Reference trajectory not set.\n";
        return false;
    }

    g_ = B_phi.transpose() * Q_ * (A_phi * x_0_ - x_ref_);
    UpdateConstrains();

    nWSR_ = 50;
    returnValue status = mpcspline_.hotstart(
        g_.data(),
        NULL,
        NULL,
        lb_C.data(),
        ub_C.data(),
        nWSR_
    );

    if (status != SUCCESSFUL_RETURN) {
        std::cerr << "[MPC] Hotstart failed! status = " << status << std::endl;
        return false;
    }

    mpcspline_.getPrimalSolution(xOpt_);
    
    Eigen::Matrix<real_t, StateDim * Horizon, 1> pred_state = A_phi * x_0_ +
        B_phi * Eigen::Map<Eigen::Matrix<real_t, InputNum * Horizon, 1>>(xOpt_);
    for (int i = 0; i < Horizon; i++) {
        prediction_pos_(i) = pred_state(i * StateDim);
    }

    return true;
}


template<int Horizon, int InputNum>
void MpcSpline<Horizon, InputNum>::debugDump() const {
    std::cout << "\n[MPC] Current x0: " << current_state_.transpose() << std::endl;
    std::cout << "[MPC] Reference: ";
    for (int i = 0; i < Horizon; i++) {
        std::cout << x_ref_(i * StateDim) << " ";
    }
    std::cout << "\n[MPC] Predicted: " << prediction_pos_.transpose() << std::endl;
}

template<int Horizon, int InputNum>
Eigen::Matrix<real_t, Horizon, 1> MpcSpline<Horizon, InputNum>::getPrediction() const 
{
    return prediction_pos_;
}

template<int Horizon, int InputNum>
Eigen::Matrix<real_t, 3, 1> MpcSpline<Horizon, InputNum>::getCurrentState() const 
{
    return current_state_;
}

// Template instantiations (example)
template class MpcSpline<3, 1>;
template class MpcSpline<5, 1>;
template class MpcSpline<10, 1>;
template class MpcSpline<20, 1>;
// template class MpcSpline<50, 1>;
