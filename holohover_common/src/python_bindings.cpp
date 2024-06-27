#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "holohover_common/models/holohover_model.hpp"

namespace py = pybind11;

PYBIND11_MODULE(bindings, m)
{
    py::class_<HolohoverProps>(m, "HolohoverProps")
        .def(py::init<>())
        .def_readwrite("mass", &HolohoverProps::mass)
        .def_readwrite("CoM", &HolohoverProps::CoM)
        .def_readwrite("inertia", &HolohoverProps::inertia)
        .def_readwrite("idle_signal", &HolohoverProps::idle_signal)
        .def_readwrite("motor_tau", &HolohoverProps::motor_tau)
        .def_readwrite("use_configuration_matrix", &HolohoverProps::use_configuration_matrix)
        .def_readwrite("configuration_matrix", &HolohoverProps::configuration_matrix)
        .def_readwrite("signal_to_thrust_coeffs_motor_1", &HolohoverProps::signal_to_thrust_coeffs_motor_1)
        .def_readwrite("signal_to_thrust_coeffs_motor_2", &HolohoverProps::signal_to_thrust_coeffs_motor_2)
        .def_readwrite("signal_to_thrust_coeffs_motor_3", &HolohoverProps::signal_to_thrust_coeffs_motor_3)
        .def_readwrite("signal_to_thrust_coeffs_motor_4", &HolohoverProps::signal_to_thrust_coeffs_motor_4)
        .def_readwrite("signal_to_thrust_coeffs_motor_5", &HolohoverProps::signal_to_thrust_coeffs_motor_5)
        .def_readwrite("signal_to_thrust_coeffs_motor_6", &HolohoverProps::signal_to_thrust_coeffs_motor_6)
        .def_readwrite("motor_pos_a_1", &HolohoverProps::motor_pos_a_1)
        .def_readwrite("motor_pos_a_2", &HolohoverProps::motor_pos_a_2)
        .def_readwrite("motor_pos_b_1", &HolohoverProps::motor_pos_b_1)
        .def_readwrite("motor_pos_b_2", &HolohoverProps::motor_pos_b_2)
        .def_readwrite("motor_pos_c_1", &HolohoverProps::motor_pos_c_1)
        .def_readwrite("motor_pos_c_2", &HolohoverProps::motor_pos_c_2)
        .def_readwrite("learned_motor_vec_a_1", &HolohoverProps::learned_motor_vec_a_1)
        .def_readwrite("learned_motor_vec_a_2", &HolohoverProps::learned_motor_vec_a_2)
        .def_readwrite("learned_motor_vec_b_1", &HolohoverProps::learned_motor_vec_b_1)
        .def_readwrite("learned_motor_vec_b_2", &HolohoverProps::learned_motor_vec_b_2)
        .def_readwrite("learned_motor_vec_c_1", &HolohoverProps::learned_motor_vec_c_1)
        .def_readwrite("learned_motor_vec_c_2", &HolohoverProps::learned_motor_vec_c_2);

    m.def("load_holohover_pros", &load_holohover_pros);

    py::class_<Holohover>(m, "Holohover")
        .def_property_readonly_static("NX", [](const py::object&) { return Holohover::NX; })
        .def_property_readonly_static("NU", [](const py::object&) { return Holohover::NU; })
        .def_property_readonly_static("NA", [](const py::object&) { return Holohover::NA; })
        .def_readonly("min_thrust", &Holohover::min_thrust)
        .def_readonly("max_thrust", &Holohover::max_thrust)
        .def_readonly("dt", &Holohover::dt)
        .def_readonly("A", &Holohover::A)
        .def_readonly("B", &Holohover::B)
        .def_readonly("Ad", &Holohover::Ad)
        .def_readonly("Bd", &Holohover::Bd)
        .def_readonly("Ad_motor", &Holohover::Ad_motor)
        .def_readonly("Bd_motor", &Holohover::Bd_motor)
        .def(py::init<HolohoverProps&, double>(), py::arg("props"), py::arg("dt") = 0.01)
        .def("nonlinear_state_dynamics", [](Holohover& holohover, const Holohover::state_t<double> &x, const Holohover::control_force_t<double> &u) {
            Holohover::state_t<double> x_dot;
            holohover.nonlinear_state_dynamics(x, u, x_dot);
            return x_dot;
        })
        .def("nonlinear_state_dynamics_force", [](Holohover& holohover, const Holohover::state_t<double> &x, const Holohover::control_force_t<double> &u) {
            Holohover::state_t<double> x_dot;
            holohover.nonlinear_state_dynamics_force(x, u, x_dot);
            return x_dot;
        })
        .def("control_force_to_acceleration", [](Holohover& holohover, const Holohover::state_t<double> &x, const Holohover::control_force_t<double> &u_force) {
            Holohover::control_acc_t<double> u_acc;
            holohover.control_force_to_acceleration(x, u_force, u_acc);
            return u_acc;
        })
        .def("control_acceleration_to_force", [](Holohover& holohover,
                                                           const Holohover::state_t<double> &x,
                                                           const Holohover::control_acc_t<double> &u_acc,
                                                           const Holohover::control_force_t<double> &min_force,
                                                           const Holohover::control_force_t<double> &max_force) {
            Holohover::control_force_t<double> u_force;
            holohover.control_acceleration_to_force(x, u_acc, u_force, min_force, max_force);
            return u_force;
        })
        .def("control_acceleration_to_force", [](Holohover& holohover,
                                                 const Holohover::state_t<double> &x,
                                                 const Holohover::control_acc_t<double> &u_acc) {
            Holohover::control_force_t<double> u_force;
            holohover.control_acceleration_to_force(x, u_acc, u_force);
            return u_force;
        })
        .def("signal_to_thrust", [](Holohover& holohover, const Holohover::control_force_t<double> &u_signal) {
            Holohover::control_force_t<double> u_thrust;
            holohover.signal_to_thrust(u_signal, u_thrust);
            return u_thrust;
        })
        .def("thrust_to_signal", [](Holohover& holohover, const Holohover::control_force_t<double> &u_thrust) {
            Holohover::control_force_t<double> u_signal;
            holohover.thrust_to_signal(u_thrust, u_signal);
            return u_signal;
        });
}
