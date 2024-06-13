% Goesta Stomberg, 2024

clear all
clc

import casadi.*

% scenario
Nrobot = 4;
N = 10;           %horizon
dt = 0.100;       %MPC sampling interval seconds
h = 0.100;

t_end = 4;
[tp,theta_ref,xref_planned,uref_planned] = three_leaved_clover_ocp(dt,t_end);

xref_planned(1,:) = xref_planned(1,:) + 0.3; %start at x = 0.6

xref_planned_basic = xref_planned;
uref_planned_basic = uref_planned;

xref_planned = []; uref_planned = [];

for k = 1:10
    xref_planned = [xref_planned, xref_planned_basic];
    uref_planned = [uref_planned, uref_planned_basic, zeros(3,1)];
end

write_clover_traj_to_csv(N,xref_planned,uref_planned,dt,10);

function write_clover_traj_to_csv(N,xref,uref,dt,t0)

    time_steps = size(xref,2) - N;
    
    %drive to starting position
    x1d = [0.6; 0.0; zeros(4,1)];  %desired setpoint for x1
    x2d = [-0.3; 0.0; zeros(4,1)];  %desired setpoint for (x2-x1)
    
    t(1) = 0;
    x(1,:) = [repmat(x1d.',1,N+1),x2d.'];
    u(1,:) = zeros(1,3*N);
    
    %clover
    for k = 1:time_steps
        t(k+1) = ceil(t0/dt) + (k-1);
        xmat = xref(:,k:k+N);
        umat = uref(:,k:k+N-1);
        x(k+1,:) = [vertcat(xmat(:)).',x2d.'];
        u(k+1,:) = vertcat(umat(:)).';
    end
    
    X = [t.',x];
    U = [t.',u];

    xdstr = sprintf("clover_traj_xd_N%i_%i.csv",N,dt*1000);
    udstr = sprintf("clover_traj_ud_N%i_%i.csv",N,dt*1000);
    
    writematrix(X,xdstr);
    writematrix(U,udstr);

end