% Goesta Stomberg, 2024

% save OCP for experiments with obstacles to file
% OCP includes quadratic collision avoidance constraints, also to obstacles

% close all
clear all
clc

import casadi.*

% scenario
Nrobot = 4;
N = 7;          %horizon
dt = 0.150;       %MPC sampling interval seconds
h = 0.150;
x10 = 1.0;
xx0 = {[x10;0.0;zeros(4,1)],[x10-0.3;0.0;zeros(4,1)],[x10-2*0.3;0.0;zeros(4,1)],[x10-3*0.3;0.0;zeros(4,1)]};
uu0 = {[0;0;0],[0;0;0],[0;0;0],[0;0;0]};

%setpoints
%easy
x1d = [-0.3; 0.0; zeros(4,1)];   %desired setpoint for x1
x2d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x2-x1)
x3d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x3-x2)
x4d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x3-x2)

xxd{1} = [x1d, x2d];
xxd{2} = [x2d, x3d];
xxd{3} = [x3d, x4d];
xxd{4} = [x4d];

xinit = []; %solver initialization

xxobs = {[0;0],[0;0],[0;0],[0;0]}; %x,y coordinates. one column for each obstacle

for i = 1:Nrobot
    dist{i} = zeros(3,1);
end

% setup OCP
sProb = holohover_sProb_QCQP_dist_obs(Nrobot,N,dt,h,xx0,uu0,xxd,xxobs,xinit,dist);

%export C code
gen_c_sProb(sProb);
