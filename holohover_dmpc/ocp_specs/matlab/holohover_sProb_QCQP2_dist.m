% MIT License
% Copyright (c) 2023 Goesta Stomberg, Henrik Ebel, Timm Faulwasser, Peter Eberhard
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
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

function sProb = holohover_sProb_QCQP2_dist(Nrobot,N,dt,h,x0,u0,xd,xinit,dist)

import casadi.*

nx = 6; %states per holohover (x,y,v_x,v_y,yaw,w_z)
nu = 3; %inputs per holohover (a_x,a_y,w_z_dot)

xmin = -inf(nx,1);
xmax = inf(nx,1);
umin = -[5;5;15]; %m/s^2, m/s^2, rad/s^2
umax =  [5;5;15];  %m/s^2, m/s^2, rad/s^2

ux =  [1;0.45]; %box constaints for position (soft constraints)
lx = -[1;0.45];

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
sysd = c2d(sysc,h);
sysdt = c2d(sysc,dt); %for riccati

Ad = sysd.A;
Bd = sysd.B;
Adt = sysdt.A;
Bdt = sysdt.B;

% declare decision variables
for i=1:Nrobot
    % states
    XX{i}  = SX.sym(['x' num2str(i)], [nx N+1]);
    ZZZ{i} = cell(1,Nrobot);
    % state copies
    s = 1:Nrobot;
    for k=s(abs(s-i)==1)
       ZZZ{i}{k} =  SX.sym(['z' num2str(i) num2str(k)], [nx N+1]);
       llbz{i}{k} = -inf(nx,N+1);
       uubz{i}{k} = inf(nx,N+1);
    end
    % inputs
    UU{i}  = SX.sym(['u' num2str(i)], [nu N]);
    % inital condition
    XX0{i} = SX.sym(['xx0' num2str(i)], [nx 1]);
    UU0{i} = SX.sym(['uu0' num2str(i)], [nu 1]);
    % setpoint
    if i < Nrobot
        XXd{i} = SX.sym(['xxd' num2str(i)], [nx 2]);
    else
        XXd{i} = SX.sym(['xxd' num2str(i)], [nx 1]);
    end
    % disturbance
    DD{i} = SX.sym(['dist' num2str(i)], [3 1]);
    slacks{i} = SX.sym(['s' num2str(i)], [1 1]); %slack for minimum-distance constraint
end


% setup individual OCPs        
for i=1:Nrobot
    JJ{i}   = 0;
    gg{i}   = [];
    hh{i}   = [];
    llbx{i} = [];
    uubx{i} = [];
    llbu{i} = [];
    uubu{i} = [];

    % over horizon ...
    
    % initial condition
    gg{i} = [gg{i}; XX{i}(:,1) - XX0{i}];
    gg{i} = [gg{i}; UU{i}(:,1) - UU0{i}];
    % dynamics
    for j=1:N
        gg{i}      = [ gg{i}; XX{i}(:,j+1) - ( Ad*XX{i}(:,j) + Bd*(UU{i}(:,j) + DD{i} ) )];
    end
    
    %terminal constraint
%     if i == 1
%         gg{i} = [gg{i}; XX{i}(:,N) - XXd{i}(:,1)];
%     else
%         gg{i} = [gg{i}; (XX{i}(:,N) - ZZZ{i}{i-1}(:,N) - XXd{i}(:,1) ) ];
%     end
    
    
    for j= 1:N+1                            
        % state and input constraints
        if j == 1 %don't enforce box constraints at initial state
            llbx{i} = [llbx{i}; -inf(nx,1)];
            uubx{i} = [uubx{i};  inf(nx,1)];
        else
            llbx{i} = [ llbx{i}, xmin];
            uubx{i} = [ uubx{i}, xmax];
        end
    end
    for j = 1:N
        if j == 1 %don't enforce box constraints at initial input
            llbu{i} = [llbu{i}; -inf(nu,1)];
            uubu{i} = [uubu{i};  inf(nu,1)];
        else
            llbu{i} = [ llbu{i}, umin];
            uubu{i} = [ uubu{i}, umax];
        end
    end
    %minimum distance constraint
    dmin = 0.3; %meter
    for j = 2:N+1
        if i > 1
            hh{i} = [hh{i}; (-(XX{i}(1:2,j) - ZZZ{i}{i-1}(1:2,j)).' * (XX{i}(1:2,j) - ZZZ{i}{i-1}(1:2,j)) + dmin^2) - slacks{i}];            
        end
        if i < Nrobot
            hh{i} = [hh{i}; (-(XX{i}(1:2,j) - ZZZ{i}{i+1}(1:2,j)).' * (XX{i}(1:2,j) - ZZZ{i}{i+1}(1:2,j)) + dmin^2) - slacks{i}];
        end
    end
    %box constraints on position
    for j = 2:N+1
        hh{i} = [hh{i}; XX{i}(1:2,j) - ux - slacks{i}];
        hh{i} = [hh{i}; lx - XX{i}(1:2,j) - slacks{i}];
    end

    JJ{i} = JJ{i} + 10^6* slacks{i}*slacks{i}.';
end

Q1 = diag([14,14,9,9,20,9]);
Qij = diag([14,14,9,9,20,9]);

R = 0.1*eye(nu);
% beta = 10;
% P = beta*eye(nx);
P1 = idare(Adt,Bdt,Q1,R);
Pij = idare(Adt,Bdt,Qij,R);

%stage cost
for i = 1:Nrobot
   ZZZ{i}{i} = XX{i};
   for k =  1:N
       if i == 1
           JJ{i} = JJ{i} + 0.5 * ( XX{i}(:,k) - XXd{i}(:,1) ).' * Q1 * ( XX{i}(:,k) - XXd{i}(:,1) );
       end

       if i > 1
           JJ{i} = JJ{i} + 0.25*( XX{i}(:,k) - ZZZ{i}{i-1}(:,k) - XXd{i}(:,1) ).'* Qij * ( XX{i}(:,k) - ZZZ{i}{i-1}(:,k) - XXd{i}(:,1) );     
       end

       if i < Nrobot
           JJ{i} = JJ{i} + 0.25*( ZZZ{i}{i+1}(:,k) - XX{i}(:,k) - XXd{i}(:,2) ).'* Qij * ( ZZZ{i}{i+1}(:,k) - XX{i}(:,k) - XXd{i}(:,2) );     
       end
       JJ{i} = JJ{i} + 0.5*UU{i}(:,k).'*R*UU{i}(:,k);
   end
end

%terminal cost
for i = 1:Nrobot
k = N+1;
    if i == 1
       JJ{i} = JJ{i} + 0.5 * ( XX{i}(:,k) - XXd{i}(:,1) ).' * P1 * ( XX{i}(:,k) - XXd{i}(:,1) );
    end

    if i > 1
       JJ{i} = JJ{i} + 0.25*( XX{i}(:,k) - ZZZ{i}{i-1}(:,k) - XXd{i}(:,1) ).'* Pij * ( XX{i}(:,k) - ZZZ{i}{i-1}(:,k) - XXd{i}(:,1) );     
    end

    if i < Nrobot
       JJ{i} = JJ{i} + 0.25*( ZZZ{i}{i+1}(:,k) - XX{i}(:,k) - XXd{i}(:,2) ).'* Pij * ( ZZZ{i}{i+1}(:,k) - XX{i}(:,k) - XXd{i}(:,2) );     
    end
end
    
        
for i=1:Nrobot
   ZZZ{i}{i} = XX{i};
   llbz{i}{i} = llbx{i};
   uubz{i}{i} = uubx{i};
   llbzi = vertcat(vertcat(llbz{i}{:}));
   uubzi = vertcat(vertcat(uubz{i}{:}));
   
   ZZZi = [];
   for j = 1:Nrobot
       if ~isempty(ZZZ{i}{j})
           for k = 1:size(ZZZ{i}{j},2)
              ZZZi = [ZZZi; ZZZ{i}{j}(:,k)]; 
           end
       end       
   end
   UUi = [];
   for k = 1:size(UU{i},2)
       UUi = [UUi; UU{i}(:,k)];
   end
   XXU{i}    = [ ZZZi; UUi];
   llbxu{i} = [llbzi(:); llbu{i}(:)];
   uubxu{i} = [uubzi(:); uubu{i}(:)];
end

% set up consensus constraints
AA = cell(1,Nrobot);
for i = 1:Nrobot
    for j = setdiff(1:Nrobot,i)
        tmp = ZZZ{i}{j};
        if ~isempty(tmp) %i has copies of j
            ncopy = numel(tmp);
            nbefore  = numel(vertcat(vertcat(ZZZ{i}{1:j-1})));            
            nafter = numel(XXU{i})-ncopy-nbefore;
            AA{i} = [AA{i}; zeros(ncopy,nbefore),-eye(ncopy),zeros(ncopy,nafter)];
            
            nbefore = numel(vertcat(vertcat(ZZZ{j}{1:j-1})));
            nafter = numel(XXU{j})-ncopy-nbefore;
            AA{j} = [AA{j}; zeros(ncopy,nbefore),eye(ncopy),zeros(ncopy,nafter)];
            
            
            for k = setdiff(1:Nrobot,[i,j])
                AA{k} = [AA{k}; zeros(ncopy,numel(XXU{k}))];
            end
                
            clear ncopy ZZZi nbefore nafter tmp
        end        
    end

    pp{i} = [XX0{i};UU0{i};DD{i};vertcat(XXd{i}(:))];

end

%account for slack variables in AA
for i = 1:Nrobot
    XXU{i} = [XXU{i};vertcat(slacks{i}(:))];
    AA{i} = [AA{i},zeros(size(AA{i},1),1)];
    llbxu{i} = [llbxu{i}; zeros(1,1)];
    uubxu{i} = [uubxu{i}; inf(1,1)];
end

% regularization
for i = 1:Nrobot
    JJ{i} = JJ{i} + 10^(-6)*XXU{i}.'*XXU{i};
end

% set up sProb
% convert expressions to MATLAB functions
% X0       = vertcat(XX0{:});

for i=1:Nrobot   
    sProb.locFuns.ffi{i} = Function(['f' num2str(i)],{XXU{i},pp{i}},{JJ{i}});
    sProb.locFuns.ggi{i} = Function(['g' num2str(i)],{XXU{i},pp{i}},{gg{i}});
    sProb.locFuns.hhi{i} = Function(['h' num2str(i)],{XXU{i},pp{i}},{hh{i}});
    
    sProb.llbx{i}  = llbxu{i};
    sProb.uubx{i}  = uubxu{i};
    sProb.AA{i}    = AA{i};
    
    if isempty(xinit)
        sProb.zz0{i}   = zeros(length(XXU{i}),1);
    else
        sProb.zz0{i} = xinit{i};
    end
    
    sProb.llam0{i} = zeros(size(AA{i},1),1);

    sProb.pp{i} = [x0{i};u0{i};dist{i};vertcat(xd{i}(:))];    
end

end