#include "dense_casadi_function_buffer.hpp"

void DenseCasadiFunctionBuffer::init(const casadi::Function& fun)
{
    function = std::make_unique<casadi::Function>(fun);
    function_buffer = std::make_unique<casadi::FunctionBuffer>(*function);

    casadi_int n_res = function->n_out();
    res.resize(n_res);
    for (casadi_int i = 0; i < n_res; i++) {
        const casadi::Sparsity& sparsity = function->sparsity_out(i);
        if (sparsity.is_dense()) {
            throw std::runtime_error("casadi function is not dense");
        }
        res[i].resize(sparsity.rows(), sparsity.columns());
        // set buffer
        function_buffer->set_res(i, res[i].data(), res[i].size());
    }
}

void DenseCasadiFunctionBuffer::set_arg(casadi_int i, const double *data, casadi_int size)
{
    assert(function_buffer != nullptr);
    function_buffer->set_arg(i, data, size);
}

void DenseCasadiFunctionBuffer::eval()
{
    assert(function_buffer != nullptr);
    function_buffer->_eval();
}
