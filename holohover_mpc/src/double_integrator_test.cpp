#include <iostream>
#include <iomanip>
#include <chrono>

#include "laopt/laopt.hpp"

#include "double_integrator_ocp.hpp"
#include "laopt/tools/multiple_shooting.hpp"
#include "laopt/solvers/sqp_solver.hpp"
#include "laopt/solvers/piqp_interface.hpp"


template<typename Transcription, typename OptProblem, typename Duration>
void print_solution(const Transcription &transcription, OptProblem &opt_problem,
                    const Duration &duration_us, const Duration &duration2_us)
{
    /* Print out the solution */
    std::cout << "\n";
    std::cout << std::setprecision(4) << std::defaultfloat;

    const Eigen::VectorXd T_opt = transcription.get_T_opt();
    const Eigen::MatrixXd X_opt = transcription.get_X_opt();
    const Eigen::MatrixXd U_opt = transcription.get_U_opt();
    const Eigen::MatrixXd p_opt = transcription.get_p_opt();
    double obj_eval = opt_problem.eval_objective();

    std::cout << "Comp. time (warm): " << duration_us / 1e3 << " (" << duration2_us / 1e3 << ") ms, tf = "
              << T_opt(T_opt.size() - 1) << " s, obj = " << obj_eval << "\n";
    std::cout << "MATLAB-copyable output:\n";
    std::cout << "T_opt = [\n" << T_opt.transpose() << "];\n";
    std::cout << "X_opt = [\n" << X_opt << "];\n";
    std::cout << "U_opt = [\n" << U_opt << "];\n";
    std::cout << "p_opt = [" << p_opt.transpose() << "];\n";
    std::cout << "obj = " << obj_eval << ";\n";
    std::cout << "comp_time = " << duration_us / 1e6 << ";\n";
    std::cout << "\n";
}

template<typename Transcription, typename Scalar>
void print_sampled_solution(const Transcription &transcription,
                            const Scalar &Ts_max = 0.01, const Scalar &t_test = 0.166)
{
    const Eigen::MatrixXd TXn = transcription.get_TX_resampled(Ts_max);
    const Eigen::MatrixXd TUn = transcription.get_TU_resampled(Ts_max);
    const Eigen::VectorXd x_test = transcription.get_x_at(t_test);
    const Eigen::VectorXd u_test = transcription.get_u_at(t_test);

    std::cout << "Resampling at " << TXn(0, 1) - TXn(0, 0) << " s  (" << Ts_max << " max)\n";
    std::cout << "MATLAB-copyable output:\n";
    std::cout << "TXn = [\n" << TXn << "];\n";
    std::cout << "TUn = [\n" << TUn << "];\n";
    std::cout << "t_test = " << t_test << ";\n";
    std::cout << "x_test = [" << x_test.transpose() << "];\n";
    std::cout << "u_test = [" << u_test.transpose() << "];\n";
    std::cout << "\n";
}

int main()
{
    using namespace std::chrono;

    /* Choose OCP and Transcription */
    using Ocp = DoubleIntegratorOcp;

    /* Construct OCP and set OCP-specific properties */
    Ocp ocp;

    ocp.set_tf(2);

    ocp.u_ub << 1, 1, 1, 1, 1, 1;
    ocp.u_lb << 0, 0, 0, 0, 0, 0;

    ocp.x_ref << 0, 0, 0, 0, 0, 0;

    ocp.set_x0({0, 0, 0, 0, 0, 0});

    /* Resampling test parameters */
    const double Ts_max = 0.02;
    const double t_test = 0.166;

    /* Solve with Multiple Shooting transcription */
    const int N = 20;
    using Transcription = laopt_tools::MultipleShooting<Ocp, N>;

    /* Define specific Tape, laOPT, and IPOPT problem types for the resulting NLP */
    using Tape = laopt::TapeInfo<Transcription>;
    using OptProblem = laopt::Problem<Transcription>;

    /* Construct transcription for OCP, optionally generate/store tape for that combination */
    Transcription transcription(ocp);
    Tape tape = laopt::generate_tape(transcription, laopt::generate_sparsity(transcription));

    /* Construct laOPT problem for transcribed OCP using according tape */
    OptProblem opt_problem(transcription, tape); // Tape is optional here and could also be generated internally

    std::cout << "Multiple Shooting - SQP\n";

    using Solver = laopt::SQPSolver<OptProblem, laopt::PIQPSolver<OptProblem::scalar_t>>;
    Solver solver(opt_problem);
    solver.settings().verbose = true;
    solver.settings().hessian_approximation = laopt::hessian_approximation_t::EXACT_NO_CONSTRAINTS;
    // solver.settings().max_watchdog_steps = 0;

    const steady_clock::time_point t_start = steady_clock::now();
    solver.solve();
    const steady_clock::time_point t_end = steady_clock::now();
    const long duration_us = duration_cast<microseconds>(t_end - t_start).count();

    solver.solve(); // Call second time to test repeatability
    const steady_clock::time_point t_end2 = steady_clock::now();
    const long duration2_us = duration_cast<microseconds>(t_end2 - t_end).count();

    /* Print out the solution */
    print_solution(transcription, opt_problem, duration_us, duration2_us);
    print_sampled_solution(transcription, Ts_max, t_test);

    return 0;
}