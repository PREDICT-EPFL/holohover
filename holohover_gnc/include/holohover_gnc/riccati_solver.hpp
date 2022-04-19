#ifndef HOLOHOVER_GNC_HOLOHOVER_RICCATI_SOLVER_HPP
#define HOLOHOVER_GNC_HOLOHOVER_RICCATI_SOLVER_HPP

// Adopted from https://github.com/TakaHoribe/Riccati_Solver

#include <Eigen/Dense>

template <typename T, int N, int M>
int solve_riccati_iteration_continuous(const Eigen::Matrix<T, N, N> &A, const Eigen::Matrix<T, N, M> &B,
                                       const Eigen::Matrix<T, N, N> &Q, const Eigen::Matrix<T, M, M> &R,
                                       Eigen::Matrix<T, N, N> &P, const double dt = 0.001,
                                       const double tolerance = 1e-5,
                                       const uint iter_max = 100000)
{
    P = Q;

    Eigen::Matrix<T, N, N> P_next;
    Eigen::Matrix<T, M, M> R_inv = R.inverse();

    double diff;
    for (uint i = 0; i < iter_max; ++i) {
        P_next = P + (P * A + A.transpose() * P - P * B * R_inv * B.transpose() * P + Q) * dt;
        diff = abs((P_next - P).maxCoeff());
        P = P_next;
        if (diff < tolerance) {
            return i + 1;
        }
    }
    return iter_max;
}

template <typename T, int N, int M>
int solve_riccati_iteration_discrete(const Eigen::Matrix<T, N, N> &Ad, const Eigen::Matrix<T, N, M> &Bd,
                                     const Eigen::Matrix<T, N, N> &Q, const Eigen::Matrix<T, M, M> &R,
                                     Eigen::Matrix<T, N, N> &P,
                                     const double tolerance = 1e-5,
                                     const uint iter_max = 100000)
{
    P = Q;

    Eigen::Matrix<T, N, N> P_next;

    double diff;
    for (uint i = 0; i < iter_max; ++i) {
        P_next = Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad + Q;
        diff = abs((P_next - P).maxCoeff());
        P = P_next;
        if (diff < tolerance) {
            return i + 1;
        }
    }
    return iter_max;
}

#endif //HOLOHOVER_GNC_HOLOHOVER_RICCATI_SOLVER_HPP
