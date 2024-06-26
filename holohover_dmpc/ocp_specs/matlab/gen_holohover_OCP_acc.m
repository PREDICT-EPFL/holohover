% MIT License
% Copyright (c) 2023 Goesta Stomberg, Henrik Ebel, Timm Faulwasser, Peter Eberhard
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

close all
clear all
clc

import casadi.*

% scenario
Nrobot = 4;
N = 7;          %horizon
dt = 0.150;       %sampling interval seconds
h = 0.150; %OCP shooting interval
xx0 = {[0.5;0;zeros(4,1)],[0.15;0.3;zeros(4,1)],[-0.15;-0.3;zeros(4,1)],[-0.5;0;zeros(4,1)]};
uu0 = {[0;0;0],[0;0;0],[0;0;0],[0;0;0]};

%setpoints
x1d = [0.3; 0.0; zeros(4,1)];  %desired setpoint for x1
x2d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x2-x1)
x3d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x3-x2)
x4d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x4-x3)

xxd{1} = [x1d, x2d];
xxd{2} = [x2d, x3d];
xxd{3} = [x3d, x4d];
xxd{4} = [x4d];

xinit = []; %solver initialization

for i = 1:Nrobot
    dist{i} = zeros(3,1);
end

% setup OCP
%sProb = holohover_sProb_acc(Nrobot,N,dt,h,xx0,uu0,xxd,xinit);
%sProb = holohover_sProb_QP2_dist(Nrobot,N,dt,h,xx0,uu0,xxd,xinit,dist);
sProb = holohover_sProb_QCQP2_dist(Nrobot,N,dt,h,xx0,uu0,xxd,xinit,dist);

%export C code
gen_c_sProb(sProb);

%meta data for yaml
nx = 6;
nu = 3;
for i = 1:Nrobot
    if i == 1 || i == Nrobot
        idx_u0(i) = 2*(N+1)*nx;
        idx_u1(i) = 2*(N+1)*nx + nu;
    else
        idx_u0(i) = 3*(N+1)*nx;
        idx_u1(i) = 3*(N+1)*nx + nu;
    end
    if i == 1
        idx_x0(i) = 0;
    else
        idx_x0(i) = (N+1)*nx;
    end
end