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

#include "sProb.hpp"

sProb::sProb() : Nagents(0) {}

//Constructor
sProb::sProb(unsigned int n_agents) : Nagents(n_agents)
{
    H.resize(n_agents);
    g.resize(n_agents);
    Aeq.resize(n_agents);
    beq.resize(n_agents);
    Aineq.resize(n_agents);
    bineq.resize(n_agents);
    A.resize(n_agents);
    ub.resize(n_agents);
    lb.resize(n_agents);
}

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

int sProb::csvRead(MatrixXd& outputMatrix, const std::string& fileName, const std::streamsize& dPrec) {
    //This method is by Carlo Cappello, see https://www.youtube.com/watch?v=m118or4f0FE
	ifstream inputData;
	inputData.open(fileName);
	cout.precision(dPrec);
	if (!inputData)
		return -1;
	string fileline, filecell;
	unsigned int prevNoOfCols = 0, noOfRows = 0, noOfCols = 0;
	while (getline(inputData, fileline)) {
		noOfCols = 0;
		stringstream linestream(fileline);
		while (getline(linestream, filecell, ',')) {
			try {
				stod(filecell);
			}
			catch (...) {
				return -1;
			}
			noOfCols++;
		}
		if (noOfRows++ == 0)
			prevNoOfCols = noOfCols;
		if (prevNoOfCols != noOfCols)
			return -1;
	}
	inputData.close();
	outputMatrix.resize(noOfRows, noOfCols);
	inputData.open(fileName);
	noOfRows = 0;
	while (getline(inputData, fileline)) {
		noOfCols = 0;
		stringstream linestream(fileline);
		while (getline(linestream, filecell, ',')) {
			outputMatrix(noOfRows, noOfCols++) = stod(filecell);
		}
		noOfRows++;
	}
	return 0;
}

void sProb::read_AA(const std::string& folderName, unsigned int Nagents){

    int error;
    MatrixXd tmp;

    for (unsigned int i = 0; i < Nagents; i++){
        std::string fileName = folderName + "/A" + std::to_string(i+1) + ".csv";
        error = csvRead(A[i],fileName,20);
    }   

    std::ignore = error;
}

void sProb::read_ublb(const std::string& folderName, unsigned int my_id_){

    int error;
    MatrixXd tmp;

    std::string fileName = "";

    unsigned int i = my_id_;
    fileName = folderName + "/ub" + std::to_string(i+1) + ".csv";
    error = csvRead(tmp,fileName,20);
    if (tmp.rows() == 0 || tmp.cols() == 0 || error !=0){
        ub[i] = VectorXd::Zero(0);
    } else {
        ub[i] = tmp.leftCols(1);
    }

    fileName = folderName + "/lb" + std::to_string(i+1) + ".csv";
    error = csvRead(tmp,fileName,20);

    if (tmp.rows() == 0 || tmp.cols() == 0 || error !=0){
        lb[i] = VectorXd::Zero(0);
    } else {
        lb[i] = tmp.leftCols(1);
    }
}
