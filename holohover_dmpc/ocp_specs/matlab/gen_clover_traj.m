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

    %load clover into horizon
    for k = 1:N+1
        t(end+1) = ceil(t0/dt) + (k-1);
        xmat = xref(:,1:1+k-1);
        umat = uref(:,1:k-1);
        x(end+1,:) = [repmat(x1d.',1,N+1-k),vertcat(xmat(:)).',x2d.'];
        u(end+1,:) = [zeros(1,3*(N-k+1)),vertcat(umat(:)).'];
    end
    
    %traverse clover
    for k = 1:time_steps
        t(end+1) = t(end)+1;
        xmat = xref(:,k:k+N);
        umat = uref(:,k:k+N-1);
        x(end+1,:) = [vertcat(xmat(:)).',x2d.'];
        u(end+1,:) = vertcat(umat(:)).';
    end

    %return and stay in starting position
    for k = 1:N
        t(end+1) = t(end)+1;
        xmat = xref(:,end-N+k:end);
        umat = uref(:,end-N+k+1:end);
        x(end+1,:) = [vertcat(xmat(:)).',repmat(x1d.',1,k),x2d.'];
        u(end+1,:) = [vertcat(umat(:)).',zeros(1,3*(k))];
    end
    t(end+1) = t(end)+1;
    x(end+1,:) = [repmat(x1d.',1,N+1),x2d.'];
    u(end+1,:) = zeros(1,3*N);
    
    X = [t.',x];
    U = [t.',u];

    xdstr = sprintf("clover_traj_xd_N%i_%i.csv",N,dt*1000);
    udstr = sprintf("clover_traj_ud_N%i_%i.csv",N,dt*1000);
    
    writematrix(X,xdstr);
    writematrix(U,udstr);

end