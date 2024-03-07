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

#include "holohover_admm_node.hpp"

//Constructor
HolohoverADMMNode::HolohoverADMMNode(const std::string& folder_name_sprob_,const int& my_id_, const int& Nagents_, const double& rho_) 
{
    my_id = my_id_;
    Nagents = Nagents_

    build_qp(folder_name_sprob_,my_id,Nagents);

    nx = sprob_.H[my_id].rows();
    ng = sprob_.Aeq[my_id].rows();
    nh = sprob_.Aineq[my_id].rows();

    std::cout << "nx = " << nx << " , " << "ng = " << ng << " , " << "nh = " << nh << std::endl;

    Ncons = sprob_.A[my_id].rows();
    rho = rho_;
    H_bar = sprob_.H[my_id] + rho * MatrixXd::Identity(nx,nx);
    g = sprob_.g[my_id];
    A = MatrixXd::Zero(ng+nh,nx);
    A.block(0,0,ng,nx) = sprob_.Aeq[my_id];
    
    if (nh > 0){
        A.block(ng,0,nh,nx) = sprob_.Aineq[my_id];
    }

    lbA = VectorXd::Zero(ng+nh);
    ubA = VectorXd::Zero(ng+nh);
    lbA.block(0,0,ng,1) = sprob_.beq[my_id];
    ubA.block(0,0,ng,1) = sprob_.beq[my_id];

    if (nh > 0){
        lbA.block(ng,0,nh,1).setConstant(-pow(10,20));  
        ubA.block(ng,0,nh,1) = sprob_.bineq[my_id];
    }

    z       = VectorXd::Zero(nx);
    z_bar   = VectorXd::Zero(nx);
    gam     = VectorXd::Zero(nx);
    ub = sprob_.ub[my_id];
    lb = sprob_.lb[my_id];

    myOptions.printLevel = qpOASES::PL_NONE; // other values: PL_NONE, PL_LOW, PL_MEDIUM, PL_HIGH, PL_TABULAR, PL_DEBUG_ITER
 
    nWSR = 1000;
    g_bar = g + gam - rho*z_bar;

    std::cout << "initializing qpOASES" << std::endl;

    loc_prob = qpOASES::QProblem(nx,ng+nh);
    loc_prob.setOptions(myOptions);
    loc_prob.init(H_bar.data(), g_bar.data(), A.data(),
                           lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);    

    isOriginal.resize(nx);
    isOriginal.setConstant(false);
    isCopy.resize(nx);
    isCopy.setConstant(false);

    out_neighbors = std::vector<int>(0);
    in_neighbors = std::vector<int>(0);
    numCopies = VectorXi::Zero(nx);    
    
    init_coupling();

    XV = new std::vector<double>[N_og];
    for (int i = 0; i < N_og; i++){
        auto idx = og_idx_to_idx.find(i);
        XV[i].resize(numCopies(idx->second)+1); //XV stores local original variable and copies from out-neighbors
    }

    //initialize comms

}

//Destructor
HolohoverControlADMMNode::~HolohoverControlADMMNode()
{
    delete[] XV; 
}

void HolohoverControlADMMNode::build_qp(const std::string& folder_name_sprob_)
{
    sprob = sProb(Nagents);
    sprob.read_AA(folder_name_sprob_,Nagents);
    sprob.read_ublb(folder_name_sprob_,my_id_);

    std::string functionLibrary = folder_name_sprob_ + "/locFuns.so";

    casadi::DM z_cas;
    casadi::DM p_cas;

    // Use CasADi's "external" to load the compiled function
    casadi::Function f; // = casadi::external("gradFun1","sProb_chain/locFuns.so");
    std::string str;

    std::vector<casadi::DM> arg; // = {z};
    std::vector<casadi::DM> res; // = f(arg);

    int nx_ = qp.A[my_id].cols();


    VectorXd z_ = Eigen::VectorXd::Zero(nx_);
  
    z_cas = Eigen2casadi(z_);
    p_cas = Eigen2casadi(p);

    //H
    str = "HessFfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    arg = {z_cas,p_cas};
    res = f(arg);
    casadi::DM H = res[0];
    MatrixXd Heig = casadi2Eigen(H);
    qp.H[my_id] = Heig;

    //g
    str = "gradFun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    qp.g[my_id] = Eigen::VectorXd::Zero(nx_);
    qp.g[my_id] = casadi2EigenVector(res[0]);

    //Aeq
    str = "JGfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    casadi::DM Aeq = res[0];
    qp.Aeq[my_id] = casadi2Eigen(Aeq);

    //Aineq
    str = "JHfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    casadi::DM Aineq = res[0];
    qp.Aineq[my_id] = casadi2Eigen(Aineq);

    //beq
    str = "eqfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    int ng_ = res[0].size1();
    int cols_ = res[0].size2();
    qp.beq[my_id] = Eigen::VectorXd::Zero(ng_);        
    std::memcpy(qp.beq[my_id].data(), res.at(0).ptr(), sizeof(double)*ng_*cols_);
    qp.beq[my_id] = -qp.beq[my_id];

    //bineq
    str = "ineqfun" + std::to_string(my_id+1); //convert to matlab index
    f = casadi::external(str,functionLibrary);
    res = f(arg);
    int nh_ = res[0].size1();
    cols_ = res[0].size2();
    qp.bineq[my_id] = Eigen::VectorXd::Zero(nh_);        
    std::memcpy(qp.bineq[m_my_id].data(), res.at(0).ptr(), sizeof(double)*nh_*cols_);
    qp.bineq[my_id] = -qp.bineq[my_id];

    VectorXd mlb = qp.lb[my_id];

    for (int k = 0; k < qp.lb[my_id].size(); k++){
        if (qp.lb[my_id][k] == -casadi::inf){
            qp.lb[my_id][k] = -pow(10,20); 
        } else if (qp.lb[my_id][k] == casadi::inf){
            qp.lb[my_id][k] = pow(10,20);
        }
    }

    for (int k = 0; k < qp.ub[my_id].size(); k++){
        if (qp.ub[my_id][k] == -casadi::inf){
            qp.ub[my_id][k] = -pow(10,20);
        } else if (qp.ub[m_my_id][k] == casadi::inf){
            qp.ub[my_id][k] = pow(10,20);
        }
    }


}


void HolohoverControlADMMNode::init_coupling()
{
    //Coupling information
    Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Coup = -MatrixXi::Ones(nx,2); //col1: owning agent ID; col2: index in nx of owning agent

    std::vector<int> nnx (Nagents);
    for (int i = 0; i < Nagents; i++){
        nnx[i] = m_sprob.A[i].cols();
    }
    MatrixXd tmp;
    std::vector<int>::iterator it;
    VectorXd nv_out = VectorXd::Zero(Nagents);

    //Identify coupling structure
    //Out-neighbors
    N_og = 0;
    for (int i = 0; i < nx; i++){
        for (int j = 0; j < Ncons; j++){
            if (m_sprob.A[my_id](j,i) >= 0.9){
                if (!isOriginal[i]){
                    isOriginal[i] = 1;
                    og_idx_to_idx.insert({N_og,i});
                    idx_to_og_idx.insert({i,N_og});
                    N_og += 1;
                    Coup(i,0) = my_id;
                    Coup(i,1) = i;
                }                
                bool isDestination = 0;
                for (int k = 0; k < Nagents; k++){
                    tmp = MatrixXd::Zero(1,nnx[k]);
                    tmp = m_sprob.A[k].block(j,0,1,nnx[k]);
                    isDestination = (tmp.array() <= -0.9).any();
                    if (isDestination){
                        it = std::find(out_neighbors.begin(), out_neighbors.end(),k);
                        nv_out[k] += 1;
                        if( it == out_neighbors.end()) {
                            out_neighbors.push_back(k);
                        }
                        break;
                    }                 
                }
            }
        }        
    }
    N_out_neighbors = out_neighbors.size();

    //In-neighbors
    bool isSource;
    VectorXd nv_in = VectorXd::Zero(Nagents);
    for (int i = 0; i < nx; i++){
        isSource = 0;
        for (int j = 0; j < Ncons; j++){
            if(m_sprob.A[my_id](j,i) <= -0.9){
                isCopy(i) = true;
                for (int k = 0; k < Nagents; k++){
                    tmp.resize(1,nnx[k]);
                    tmp = m_sprob.A[k].block(j,0,1,nnx[k]);
                    isSource = (tmp.array() >= 0.9).any();
                    if (isSource){
                        isSource = 0;
                        Coup(i,0) = k;
                        for (int l = 0; l < m_sprob.A[k].cols(); l++){
                            if (m_sprob.A[k](j,l) == 1){
                                Coup(i,1) = l;
                                break;
                            }
                        }
                        nv_in[k] += 1;
                        it = std::find(in_neighbors.begin(), in_neighbors.end(),k);
                        if( it == in_neighbors.end()) {
                            in_neighbors.push_back(k);
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
    N_in_neighbors = in_neighbors.size();
    isCopy.resize(nx);

    for (int i = 0; i < nx; i++){
        if (!isOriginal(i) && !isCopy(i)){
            isOriginal(i) = true;
            og_idx_to_idx.insert({N_og,i});
            idx_to_og_idx.insert({i,N_og});
            Coup(i,0) = my_id;
            Coup(i,1) = i;
            N_og += 1;
        }
    }


    //Setup v_in and v_out
    v_in = new vCpy[N_in_neighbors];
    v_out = new vCpy[N_out_neighbors];

    //In-neighbors
    int nij = 0;
    for (int j = 0; j < N_in_neighbors; j++){
        nij = 0;
        for (int i = 0; i < nx; i++){
            if (Coup(i,0) == in_neighbors[j]){ //Variable i was copied from agent j
                nij+=1;
            }
        }
        v_in[j].original_agent = in_neighbors[j];
        v_in[j].copying_agent = my_id;
        v_in[j].val = VectorXd::Zero(nij);
        v_in[j].cpy_idx = VectorXi::Zero(nij);
        v_in[j].og_idx = VectorXi::Zero(nij);
        v_in[j].nv = nij;
        nij = 0;
        for (int i = 0; i < nx; i++){
            if (Coup(i,0) == in_neighbors[j]){ //Variable i was copied from agent j
                v_in[j].cpy_idx(nij) = i;
                v_in[j].og_idx(nij) = Coup(i,1);
                v_in[j].og_idx_to_cpy_idx.insert({v_in[j].og_idx(nij),v_in[j].cpy_idx(nij)});
                nij+=1;
            }
        }
    }

    //Out-neighbors
    for (int j = 0; j < N_out_neighbors; j++){
        v_out[j].copying_agent = out_neighbors[j];
        v_out[j].original_agent = my_id;
        v_out[j].nv = nv_out[out_neighbors[j]];
        v_out[j].val = VectorXd::Zero(v_out[j].nv);
        v_out[j].cpy_idx = VectorXi::Zero(v_out[j].nv);
        v_out[j].og_idx = VectorXi::Zero(v_out[j].nv);
        int k = out_neighbors[j];

        int nij = 0;
        for (int i = 0; i < nx; i++){
            bool isDestination = 0;
            for (int c = 0; c < Ncons; c++){
                if (m_sprob.A[my_id](c,i) == 1){
                    tmp.resize(1,nnx[k]);
                    tmp = m_sprob.A[k].block(c,0,1,nnx[k]);
                    isDestination = (tmp.array() == -1.0).any();
                    if (isDestination){
                        for (int l = 0; l < nnx[k]; l++){
                            if (tmp(0,l) == -1){
                                v_out[j].cpy_idx[nij] = l;
                                v_out[j].og_idx[nij] = i;
                                numCopies[i] += 0;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    return;
}

int HolohoverControlADMMNode::solve(unsigned int maxiter_, Eigen::VectorXd& zbar_)
{
    zbar = zbar_;
    for (unsigned int iter = 0; iter < maxiter_; iter++){

        //std::cout << "Starting ADMM iteration " << iter << std::endl;

        //Step 1: local z update
        g_bar = g + gam - rho*zbar;
        nWSR = 10000;
        loc_prob.hotstart(g_bar.data(),
                           lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);
        loc_prob.getPrimalSolution(z.data());
        for (int i = 0; i < N_og; i++){
            auto idx = og_idx_to_idx.find(i);
            XV[i].clear();
            XV[i].push_back(z(idx->second));
        }
        
        //Communication
        update_v_in();
        send_vin_receive_vout(0);

        //Step 2: averaging
        double avg;
        for (int i = 0; i < N_og; i++){
            avg = std::accumulate(XV[i].begin(), XV[i].end(), 0.0) / XV[i].size();
            auto idx = og_idx_to_idx.find(i);
            z_bar[idx->second] = avg;
        }

        //Communication
        update_v_out();
        send_vout_receive_vin(0);

        //Step 3: dual update
        gam = gam + rho*(z-z_bar);  

    }

    zbar_ = z_bar; //update output variable
    
    return 0;
}

void HolohoverControlADMMNode::send_vin_receive_vout(int64_t seq_number_){
    //send copies in vin and receive copies in vout
    //this is done before the averaging step

    //send vin
    //send_vin_timer.tic();
    for (int i = 0; i < N_in_neighbors; i++){
        v_inMsg_old[i].seq_number = v_inMsg[i].seq_number;
        v_inMsg_old[i].value = v_inMsg[i].value;
        v_inMsg[i].seq_number += 1;
        Eigen::VectorXd::Map(&v_inMsg[i].value[0], v_in[i].val.size()) = v_in[i].val;
        comms->publish(v_inTopic[i],&v_inMsg[i]);
    }
    //send_vin_timer.toc();
    //receive vout
    //receive_vout_timer.tic();
    Eigen::Array<bool,Dynamic,1> received(N_out_neighbors,1);
    received.fill(false);
    int counter = 0;
    int og_idx = 0;
    int cpy_id = 0;
    int idx = 0;
    while (!received.all()){
        for (int i = 0; i < N_out_neighbors; i++){
            if (!received(i)){
                if (v_outHandlers[i]->processMessage()){
                    received(i) = true;
                    for (int j = 0; j < v_outHandlers[i]->val_length; j++){
                        idx = v_outMsg_idx_firstReceived[i][j];
                        auto og_idx = idx_to_og_idx.find(idx);
                        if (og_idx != idx_to_og_idx.end()){
                            XV[og_idx->second].push_back(v_outHandlers[i]->value[j]);
                        }
                    }
                }
            }
        }
    }
    //receive_vout_timer.toc();


    return;
}

void HolohoverControlADMMNode::send_vout_receive_vin(int64_t seq_number_){
    //send originals in vout and receive copies in vin
    //this is done after the averaging step

    //send vout
    for (int i = 0; i < N_out_neighbors; i++){
        v_outMsg_old[i].seq_number = v_outMsg[i].seq_number;
        v_outMsg_old[i].value = v_outMsg[i].value;
        v_outMsg[i].seq_number += 1;

        int idx_row = 0;
        for (int j = 0; j < nx; j++){
            if (isOriginal[j]){
                v_outMsg[i].value[idx_row] = z_bar[j];
                idx_row += 1;
            }
            if (idx_row == v_outMsg[i].val_length){
                break;        
            }
        }
        comms->publish(v_outTopic[i],&v_outMsg[i]);
    }

    //receive vin
    //attention: save received value directly to z_bar (not to v_in!)
    Eigen::Array<bool, Dynamic, 1> received(N_in_neighbors,1);
    received.fill(false);
    int counter = 0;
    int cpy_idx = 0;
    int og_idx = 0;
    while (!received.all()){
        for (int i = 0; i < N_in_neighbors; i++){
            if (!received(i)){
                if (v_inHandlers[i]->processMessage()){
                    received(i) = true;
                    for (int j = 0; j < v_inHandlers[i]->val_length; j++){
                        og_idx = v_inMsg_idx_firstReceived[i][j];
                        auto tmp = v_in[i].og_idx_to_cpy_idx.find(og_idx);
                        if (tmp != v_in[i].og_idx_to_cpy_idx.end()){
                            cpy_idx = tmp->second;
                            z_bar[cpy_idx] = v_inHandlers[i]->value[j];
                        }
                        else{
                        }  
                    }
                }
            }
        }
    }
    return;
}


void HolohoverControlADMMNode::update_v_in(){
    //update all v_in values with values from z
    for (int i = 0; i < N_in_neighbors; i++){
        for (int j = 0; j < v_in[i].nv; j++){
            v_in[i].val[j] = z[v_in[i].cpy_idx[j]];
        }
    }

    return;
}

void HolohoverControlADMMNode::update_v_out(){
    //update all v_out values with values from z_bar
    for (int i = 0; i < N_out_neighbors; i++){
        for (int j = 0; j < v_out[i].nv; j++){
            v_out[i].val[j] = z_bar[v_out[i].og_idx[j]];
        }
    }
}


int HolohoverControlADMMNode::update_g_beq(){
    g = m_sprob.g[my_id];    
    lbA.block(0,0,ng,1) = m_sprob.beq[my_id];
    ubA.block(0,0,ng,1) = m_sprob.beq[my_id];
    return 0;
}



