#ifndef SPARSE_CASADI_FUNCTION_BUFFER_HPP
#define SPARSE_CASADI_FUNCTION_BUFFER_HPP

#include <Eigen/Sparse>
#include "casadi/casadi.hpp"

class SparseCasadiFunctionBuffer
{
public:
    std::vector<Eigen::SparseMatrix<double>> res;

    void init(const casadi::Function& function);
    void set_arg(casadi_int i, const double* data, casadi_int size);
    void eval();
private:
    std::unique_ptr<casadi::Function> function;
    std::unique_ptr<casadi::FunctionBuffer> function_buffer;
};


#endif //SPARSE_CASADI_FUNCTION_BUFFER_HPP
