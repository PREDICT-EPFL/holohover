#include "sparse_casadi_function_buffer.hpp"

void SparseCasadiFunctionBuffer::init(const casadi::Function& fun)
{
    function = std::make_unique<casadi::Function>(fun);
    function_buffer = std::make_unique<casadi::FunctionBuffer>(*function);

    casadi_int n_res = function->n_out();
    res.resize(n_res);
    for (casadi_int i = 0; i < n_res; i++) {
        const casadi::Sparsity& sparsity = function->sparsity_out(i);
        casadi_int rows = sparsity.rows();
        casadi_int cols = sparsity.columns();
        res[i].resize(rows, cols);

        // preallocate nnz
        Eigen::VectorXi col_nnz(cols);
        for (casadi_int col = 0; col < cols; col++) {
            col_nnz[col] = (int) (sparsity.colind()[col + 1] - sparsity.colind()[col]);
        }
        res[i].reserve(col_nnz);

        // insert dummy values
        for (casadi_int col = 0; col < cols; col++) {
            for (casadi_int row_idx = sparsity.colind()[col]; row_idx < sparsity.colind()[col + 1]; row_idx++) {
                casadi_int row = sparsity.row()[row_idx];
                res[i].insert(row, col) = 1.0; // insert dummy value for now
            }
        }
        res[i].makeCompressed();
        // set buffer
        function_buffer->set_res(i, res[i].valuePtr(), res[i].nonZeros() * sizeof(double));
    }
}

void SparseCasadiFunctionBuffer::set_arg(casadi_int i, const double *data, casadi_int size)
{
    assert(function_buffer != nullptr);
    function_buffer->set_arg(i, data, size * sizeof(double));
}

void SparseCasadiFunctionBuffer::eval()
{
    assert(function_buffer != nullptr);
    function_buffer->_eval();
}
