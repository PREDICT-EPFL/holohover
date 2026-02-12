#ifndef HOLOHOVER_GNC_HOLOHOVER_OCP_HPP
#define HOLOHOVER_GNC_HOLOHOVER_OCP_HPP

// End user (level 1)

#include <Eigen/Dense>

#include "laopt/laopt.hpp"
#include "laopt/tools/control_problem_base.hpp"

#include "holohover_common/models/holohover_model.hpp"

#include "control_mpc_settings.hpp"

class HolohoverOcp : public laopt_tools::ControlProblemBase</*Scalar*/ double, /*NX*/ 6, /*NU*/ 6>
{
public:
    /* Static parameters */

    State x_ref{0, 0, 0, 0, 0, 0};

    Eigen::Vector<Scalar, NX> Q;
    Eigen::Vector<Scalar, NU> R;

    Holohover holohover;

    HolohoverOcp(ControlMPCSettings &control_settings, Holohover &holohover_) : holohover(holohover_)
    {
        Q << control_settings.weight_x, control_settings.weight_y,
             control_settings.weight_v_x, control_settings.weight_v_y,
             control_settings.weight_yaw, control_settings.weight_w_z;
        R << control_settings.weight_motor, control_settings.weight_motor, control_settings.weight_motor,
             control_settings.weight_motor, control_settings.weight_motor, control_settings.weight_motor;
    
    };

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

        state_t<T> x_dot;
        holohover.template nonlinear_state_dynamics<T>(x, u, x_dot);

        return x_dot;
    }

};

#endif //HOLOHOVER_GNC_HOLOHOVER_OCP_HPP
