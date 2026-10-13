% Goesta Stomberg, 2024

% close all
clear all
clc

import casadi.*

% scenario
Nrobot = 4;
N = 20;          %horizon
dt = 0.050;       %MPC sampling interval seconds
h = 0.050;
x10 = 0.6;
xx0 = {[x10;0.0;zeros(4,1)],[x10-0.3;0.0;zeros(4,1)],[x10-2*0.3;0.0;zeros(4,1)],[x10-3*0.3;0.0;zeros(4,1)]};
uu0 = {[0;0;0],[0;0;0],[0;0;0],[0;0;0]};

%setpoints
%easy
x1d = [0.6; 0.0; zeros(4,1)];  %desired setpoint for x1
x2d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x2-x1)
x3d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x3-x2)
x4d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x4-x3)

t_end = (N+1)*dt;
[tp,theta_ref,xref_planned,uref_planned] = three_leaved_clover_ocp(dt,t_end);


xref_planned(1,:) = xref_planned(1,:) + 0.3; %start at x = 0.6

xref = xref_planned(:,1:N+1);
uref = uref_planned(:,1:N);

xxd{1} = [xref, x2d];
xxd{2} = [x2d, x3d];
xxd{3} = [x3d, x4d];
xxd{4} = [x4d];

xinit = []; %solver initialization

for i = 1:Nrobot
    dist{i} = zeros(3,1);
end

% setup OCP
% sProb = holohover_sProb_QP_traj(Nrobot,N,dt,h,xx0,uu0,xxd,uref,xinit);
sProb = holohover_sProb_QCQP_traj(Nrobot,N,dt,h,xx0,uu0,xxd,uref,xinit,dist);

gen_c_sProb( sProb );

%%
nx = 6; nu = 3;
for i = 1:Nrobot
    if i == 1 
        nxd(i) = nx*(N+1) + nx;
    elseif i == Nrobot
        nxd(i) = 6;
    else
        nxd(i) = 12;
    end
    nud(i) = N*nu;
end
nxd
nud