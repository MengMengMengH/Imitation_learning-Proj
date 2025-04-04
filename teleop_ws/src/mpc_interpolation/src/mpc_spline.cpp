#include "../include/mpc_interpolation/mpc_spline.hpp"

template <int Horizon, int InputNum>
MpcSpline<Horizon,InputNum>::MpcSpline(double dt, size_t Index) :
    mpcspline_(InputNum * Horizon, 3 * Horizon),
    xOpt_(new real_t[InputNum * Horizon])
{
    assert(Index < 7 && "This implementation supports for 7 joints only");
    constrain_index = Index;
    //define A and B matrix
    A_ << 1.0, dt,  dt * dt / 2.0, dt * dt * dt / 6.0,
          0.0, 1.0, dt,            dt * dt / 2.0,
          0.0, 0.0, 1.0,           dt,
          0.0, 0.0, 0.0,           1.0;
    
    B_ << dt * dt * dt *dt /24.0,
          dt * dt * dt / 6.0,
          dt * dt / 2.0,
          dt;
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
        Q_.block(i * StateDim, i * StateDim, StateDim, StateDim).diagonal() << 1.0e4, 0, 0, 0;
        R_(i, i) = 1.0;
    }
    // std::cout<<"Q_ = \n" << Q_ << std::endl;
    // compute H and g
    H_ = (B_phi.transpose() * Q_ * B_phi + R_);
    g_ = B_phi.transpose() * Q_ * (A_phi * x_0_ - x_ref_);

    // QP constraints
    C_.setZero();
    lb_C.setZero();
    ub_C.setZero();
    // std::cout << "B_phi = \n" << B_phi << std::endl;
    for (int i = 0; i < Horizon; i++)
    {
        // velocity constraints
        C_.block(i, 0, 1, InputNum * Horizon) = B_phi.block(i * StateDim + 1, 0, 1, InputNum * Horizon);
        lb_C(i) = vMin[constrain_index] - (A_phi.block(i * StateDim + 1, 0, 1, StateDim) * x_0_)(0);
        ub_C(i) = vMax[constrain_index] - (A_phi.block(i * StateDim + 1, 0, 1, StateDim) * x_0_)(0);

        // acceleration constraints
        C_.block(Horizon + i, 0, 1, InputNum * Horizon) = B_phi.block(i * StateDim + 2, 0, 1, InputNum * Horizon);
        lb_C(Horizon + i) = aMin[constrain_index] - (A_phi.block(i * StateDim + 2, 0, 1, StateDim) * x_0_)(0);
        ub_C(Horizon + i) = aMax[constrain_index] - (A_phi.block(i * StateDim + 2, 0, 1, StateDim) * x_0_)(0);

        // jerk constraints
        C_.block(2 * Horizon + i, 0, 1, InputNum * Horizon) = B_phi.block(i * StateDim + 3, 0, 1, InputNum * Horizon);
        lb_C(2 * Horizon + i) = jMin[constrain_index] - (A_phi.block(i * StateDim + 3, 0, 1, StateDim) * x_0_)(0);
        ub_C(2 * Horizon + i) = jMax[constrain_index] - (A_phi.block(i * StateDim + 3, 0, 1, StateDim) * x_0_)(0);
    }
    // std::cout << "A_phi = \n" << A_phi << std::endl;
    // std::cout << "C_ = \n" << C_ << std::endl;

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
void MpcSpline<Horizon, InputNum>::UpdateX(Eigen::Matrix<real_t, 4, 1> x0) {
    x_0_ = x0;
}

template<int Horizon, int InputNum>
void MpcSpline<Horizon, InputNum>::UpdateXRef(double x_ref) {
    for(int i = 0; i < Horizon - 1; i++) {
        x_ref_.block(i * StateDim, 0, StateDim, 1) = x_ref_.block((i + 1) * StateDim, 0, StateDim, 1);
    }
    x_ref_.block((Horizon - 1) * StateDim, 0, 1, 1) << x_ref;  // Update position reference only
}

template<int Horizon, int InputNum>
void MpcSpline<Horizon, InputNum>::SetZeroXRef() {
    x_ref_.setZero();
}

template<int Horizon, int InputNum>
void MpcSpline<Horizon, InputNum>::UpdateConstrains() {
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
        // jerk constraints
        lb_C(2 * Horizon + i) = jMin[constrain_index] - (A_phi.block(i * StateDim + 3, 0, 1, StateDim) * x_0_)(0);
        ub_C(2 * Horizon + i) = jMax[constrain_index] - (A_phi.block(i * StateDim + 3, 0, 1, StateDim) * x_0_)(0);
    }
}

template<int Horizon, int InputNum>
Eigen::Matrix<real_t, InputNum * Horizon, 1> 
    MpcSpline<Horizon, InputNum>::RenewDeltaJerk() {
    // std::cout << "\nx_ref = " << x_ref_.transpose() << std::endl;
    g_ = B_phi.transpose() * Q_ * (A_phi * x_0_ - x_ref_);
    // std::cout << "\nx_ref = " << x_ref_.transpose() << std::endl;
    UpdateConstrains();
    // std::cout << "lb_C = " << lb_C.transpose() << std::endl;
    // std::cout << "ub_C = " << ub_C.transpose() << std::endl;
    nWSR_ = 10;

    returnValue status = mpcspline_.hotstart(
        g_.data(),
        NULL,
        NULL,
        lb_C.data(),
        ub_C.data(),
        nWSR_
    );
    if (status != SUCCESSFUL_RETURN) {
        printf("hotstart failed with return value %d\n", status);
    }
    mpcspline_.getPrimalSolution( xOpt_ );

    // std::cout << "预测状态: \n" << (A_phi * x_0_ + B_phi * Eigen::Map<Eigen::Matrix<real_t, InputNum * Horizon, 1>>(xOpt_)).transpose() << std::endl;
    for (int i = 0; i < InputNum * Horizon; i++) {
        u_(i) = xOpt_[i];
    }
    return u_;
}

template<int Horizon, int InputNum>
Eigen::Matrix<real_t, 4, 1> 
MpcSpline<Horizon, InputNum>::getSimulateState() 
{
    return A_ * x_0_ + B_ * xOpt_[0];
}

// Template instantiations (example)
template class MpcSpline<3, 1>;
template class MpcSpline<5, 1>;
template class MpcSpline<10, 1>;
template class MpcSpline<20, 1>;
// template class MpcSpline<50, 1>;




