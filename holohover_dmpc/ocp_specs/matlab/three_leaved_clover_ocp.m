function [t,theta,x,u] = three_leaved_clover_ocp(dt,t_end)
    
N = t_end/dt - 1;

Ac = [0,0,1,0,0,0;
  0,0,0,1,0,0;
  0,0,0,0,0,0;
  0,0,0,0,0,0;
  0,0,0,0,0,1;
  0,0,0,0,0,0];
Bc = [0,0,0;
  0,0,0;
  1,0,0;
  0,1,0;
  0,0,0;
  0,0,1];
C =  [1,0,0,0,0,0;
  0,1,0,0,0,0;
  0,0,0,0,1,0];
D = zeros(3,3);
sysc = ss(Ac,Bc,C,D);
sysd = c2d(sysc,dt);

Ad = sysd.A;
Bd = sysd.B;

nx = 6;
nu = 3;

import casadi.*
theta  = SX.sym(['theta'], [1, N+1]); % path variable
x      = SX.sym(['x'], [nx, N+1]);  % state
u      = SX.sym(['u'], [nu, N]);    % input

x0 = [1;zeros(5,1)];
u0 = zeros(nu,1);

Q = 5;
R = 15;
J = 0;
for k = 1:N+1
    J = J + 0.5*Q*theta(k)^2;
end

for k = 1:N
    J = J + 0.5*u(:,k).'*R*u(:,k);
end

% constraints
g = [];

% initial condition
g = [g; x(3:6,1) - x0(3:6)];
g = [g; u(:,1) - u0];
g = [g; theta(:,1) + pi];


h = [];
% dynamics
for k = 1:N
    g = [g; x(:,k+1) - ( Ad*x(:,k) + Bd*u(:,k) )];
end

% path constraints
rho = 0.3;
for k = 1:N+1
    g = [g; x(1,k) - rho*sin(3*theta(k) - 1.5*pi)*cos(theta(k))]; %x
    g = [g; x(2,k) - rho*sin(3*theta(k) - 1.5*pi)*sin(theta(k))]; %y
end

for k = 1:N
    h = [h; theta(k) - theta(k+1)];
end

% terminal constraint
g = [g; theta(end)];
g = [g; x(3:4,end)];

lbx = []; ubx = [];
lbu = []; ubu = [];
lbtheta = []; ubtheta = [];
% box constraints
for k = 1:N+1
    lbx = [lbx;-inf(nx,1)];
    ubx = [ubx; inf(nx,1)];
    lbtheta = [lbtheta; -pi];
    ubtheta = [ubtheta;  0 ];
end

for k = 1:N
    lbu = [lbu; -4*ones(nu,1)];
    ubu = [ubu; 4*ones(nu,1)];
end



% solve
nlp_opts.ipopt.print_level = 5;

tlc = three_leaved_clover_fun(rho,t_end);
t = 0:dt:(t_end-dt);

for k = 1:length(t)
    px(k) = full(tlc.x(t(k)));
    py(k) = full(tlc.y(t(k)));
    vx(k) = full(tlc.vx(t(k)));
    vy(k) = full(tlc.vy(t(k)));
    ax(k) = full(tlc.ax(t(k)));
    ay(k) = full(tlc.ay(t(k)));
    vec(k) = sqrt(vx(k)^2 + vy(k)^2);
    acc(k) = sqrt(ax(k)^2 + ay(k)^2);
end

state0 = [px;py;vx;vy;zeros(1,length(t));zeros(1,length(t))];
input0 = [ax(1:end-1); ay(1:end-1); zeros(1,length(t)-1)];
theta0 = -pi + pi/t_end*t;

x0 = [theta0.'; vertcat(state0(:)); vertcat(input0(:))];

% x0 = zeros(N+1+nx*(N+1)+nu*N,1);

lb = [lbtheta;lbx;lbu];
ub = [ubtheta;ubx;ubu];

nlp = struct('x',[vertcat(theta(:));vertcat(x(:));vertcat(u(:))],'f',J,'g',[g;h]);
cas = nlpsol('solver','ipopt',nlp,nlp_opts);
tic
sol = cas('x0' , x0,...
          'lbx', lb,...
          'ubx', ub,...
          'lbg', [zeros(length(g),1); -inf*ones(length(h),1)], ...
          'ubg', [zeros(length(g),1);  zeros(length(h),1)]);
toc

xopt    = full(sol.x);  

x = reshape(xopt(N+1+1:N+1+nx*(N+1)),nx,[]);
u = reshape(xopt(N+1+nx*(N+1)+1: N+1+nx*(N+1)+nu*N),nu,[]);
theta = reshape(xopt(1:N+1),1,[]);

end


function tlc = three_leaved_clover_fun(rho,t_end)

    import casadi.*
    t  = SX.sym(['t'], [1]); % time
    
    theta = -pi + pi/t_end*t;
    
    
    x = rho*sin(3*theta - 1.5*pi)*cos(theta);
    y = rho*sin(3*theta - 1.5*pi)*sin(theta);
    
    xFun = Function(['x'],{t},{x});
    yFun = Function(['y'],{t},{y});
    
    vxCas    = jacobian(xFun(t), t);
    vxFun = Function('vxFun',{t},{vxCas});
    
    axCas = jacobian(vxFun(t), t);
    axFun = Function('axFun',{t},{axCas});

    vyCas    = jacobian(yFun(t), t);
    vyFun = Function('vyFun',{t},{vyCas});
    
    ayCas = jacobian(vyFun(t), t);
    ayFun = Function('ayFun',{t},{ayCas});
    
    tlc.x = xFun;
    tlc.vx = vxFun;
    tlc.ax = axFun;
    tlc.y = yFun;
    tlc.vy = vyFun;
    tlc.ay = ayFun;

end

