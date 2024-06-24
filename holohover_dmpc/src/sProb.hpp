/*MIT License
Copyright (c) 2023 Goesta Stomberg, Henrik Ebel, Timm Faulwasser, Peter Eberhard
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef SPROB_H
#define SPROB_H

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <string>

#include "sparse_casadi_function_buffer.hpp"
#include "dense_casadi_function_buffer.hpp"

/*
QP definition:

min sum_i 0.5*x[i].'*H[i]*x[i] + g[i].'x[i]

s.t.
    Aeq[i]*x[i] = beq[i]
    Aineq[i]*x[i] <= bineq[i]
    sum_i A[i]x[i] = 0
*/

class sProb
{
public:
    unsigned int Nagents;

    std::vector<SparseCasadiFunctionBuffer> H;
    std::vector<DenseCasadiFunctionBuffer> g;
    std::vector<SparseCasadiFunctionBuffer> Aeq;
    std::vector<DenseCasadiFunctionBuffer> beq;
    std::vector<SparseCasadiFunctionBuffer> Aineq;
    std::vector<DenseCasadiFunctionBuffer> bineq;
    std::vector<Eigen::MatrixXd> A;
    std::vector<Eigen::VectorXd> ub;
    std::vector<Eigen::VectorXd> lb;

    sProb();
    explicit sProb(unsigned int );

    int csvRead(Eigen::MatrixXd& outputMatrix, const std::string& fileName, const std::streamsize& dPrec);
    void read_AA(const std::string& folderName, unsigned int Nagents);
    void read_ublb(const std::string& folderName, unsigned int my_id_);
};

#endif