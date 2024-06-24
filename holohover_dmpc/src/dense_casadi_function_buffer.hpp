#ifndef DENSE_CASADI_FUNCTION_BUFFER_HPP
#define DENSE_CASADI_FUNCTION_BUFFER_HPP

#include <Eigen/Dense>
#include "casadi/casadi.hpp"

class DenseCasadiFunctionBuffer
{
public:
    std::vector<Eigen::MatrixXd> res;

    void init(const casadi::Function& function);
    void set_arg(casadi_int i, const double* data, casadi_int size);
    void eval();
private:
    std::unique_ptr<casadi::Function> function;
    std::unique_ptr<casadi::FunctionBuffer> function_buffer;
};


#endif //DENSE_CASADI_FUNCTION_BUFFER_HPP
