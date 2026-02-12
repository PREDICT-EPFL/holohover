#ifndef LAOPT_DOUBLE_INTEGRATOR_OCP_HPP
#define LAOPT_DOUBLE_INTEGRATOR_OCP_HPP

// End user (level 1)

#include <Eigen/Dense>

#include "laopt/laopt.hpp"
#include "laopt/tools/control_problem_base.hpp"

#include "holohover_common/models/holohover_model.hpp"


class DoubleIntegratorOcp : public laopt_tools::ControlProblemBase</*Scalar*/ double, /*NX*/ 6, /*NU*/ 6>
{
public:
    /* Static parameters */
    // Eigen::Matrix<Scalar, NX, NX> A{{0, 1},
    //                                 {0, 0}};
    // Eigen::Matrix<Scalar, NX, NU> B{{0},
    //                                 {1}};

    State x_ref{0, 0, 0, 0, 0, 0};
    Eigen::Vector<Scalar, NX> Q{14, 14, 1.0, 1.0, 1.0, 1.0};

    Eigen::Vector<Scalar, NU> R{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    HolohoverProps holohover_props;
    Holohover holohover;

    DoubleIntegratorOcp() : holohover(holohover_props) {};

    /* Override function implementations from base class ------------------------------ */
    template<typename T> // T is scalar type
    T lagrange_term_impl(const Eigen::Ref<const state_t<T>> &x,
                         const Eigen::Ref<const input_t<T>> &u,
                         const Eigen::Ref<const param_t<T>> &p)
    {
        return (x_ref - x).dot(Q.asDiagonal() * (x_ref - x)) + u.dot(R.asDiagonal() * u);
    }

    template<typename T, typename Ttf> // T is scalar type
    T mayer_term_impl(const Eigen::Ref<const state_t<T>> &xf,
                      const Eigen::Ref<const param_t<T>> &p,
                      const Ttf &tf)
    {
        return (x_ref - xf).dot(Q.asDiagonal() * (x_ref - xf));
    }

    template<typename T> // T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        // state_t<T> x_dot = A * x + B * u;
        state_t<T> x_dot;
        holohover.template nonlinear_state_dynamics<T>(x, u, x_dot);
        return x_dot;
    }

};

#endif //LAOPT_DOUBLE_INTEGRATOR_OCP_HPP
