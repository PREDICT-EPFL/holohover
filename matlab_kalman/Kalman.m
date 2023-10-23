%% Convention
% x_k_k denotes x_k|k and x_k_k1 denotes x_k|k-1
%% Setup
clear ; clc ; %close all
cd data/data/
imu = readtable("drone_imu.csv");
optitrack = readtable("optitrack_drone_pose.csv");
mouse = readtable("drone_mouse.csv");
cd .. ; cd ..;

optitrack.time = optitrack.header_stamp_sec + optitrack.header_stamp_nanosec * 10^-9;
imu.time = imu.header_stamp_sec + imu.header_stamp_nanosec * 10^-9;
mouse.time = mouse.header_stamp_sec + mouse.header_stamp_nanosec * 10^-9;

optitrack_plot = optitrack;
optitrack = optitrack(1:30:end,:);

%% List creation (not usefull for now)
imu_mes = [imu.time, ones(length(imu.time),1)];
opt_mes = [optitrack.time, 2*ones(length(optitrack.time),1)];
mouse_mes = [mouse.time, 3*ones(length(mouse.time),1)];

mes = [imu_mes; opt_mes; mouse_mes];
%mes = [imu_mes; opt_mes];
mes = sortrows(mes);

%% Init

R1 = 0.25; % IMU update
R2 = [0.001, 0, 0; % Optitrack update
      0, 0.001, 0;
      0, 0, 0.05];
R3 = [0.05, 0; % Mouse update
      0, 0.05];

Q = diag([ones(3,1)*1e-2; ones(2,1)*1e-1; 1.0; ones(2,1)*1e-2]);


P_k_k = Q;

std{1} = (Standard_deviation(P_k_k))'; % A cleaner way to store them for the plotting

x_k_k{1} = [optitrack.pose_position_x(1); 
            optitrack.pose_position_y(1);
            optitrack.pose_orientation_yaw(1); 
            0; 0; 0; 
            -imu.acc_x(1); 
            -imu.acc_y(1)];
x_k_k1 = x_k_k;
acc{1} = [0;0];
j = 0; k = 0; i = 1; % i is the index of the IMU, j is for the optitrack, k is for the mouse l is the sum of them
time(1) = imu.time(1); % l & i starts at 2 because we initialize with imu(1)

%% Filtering the data
for l = 2:size(mes,1)
    switch(mes(l,2))
        case 1 % imu
            i = i + 1; dt = imu.time(i) - time(l-1); time(l) = imu.time(i); 
            [x_k_k1{l},P_k_k1,acc{l},gamma] = Kalman_predict(P_k_k,Q,x_k_k{l-1},i,imu,dt);
            z = imu.gyro_z(i);
            [x_k_k{l},P_k_k] = Kalman_update(P_k_k1,gamma,R1,x_k_k1{l},z,"imu",j,k);
            continue;
        case 2 % optitrack
            j = j + 1; dt = optitrack.time(j)- time(l-1); time(l) = optitrack.time(j);
            [x_k_k1{l},P_k_k1,acc{l},gamma] = Kalman_predict(P_k_k,Q,x_k_k{l-1},i,imu,dt);
            z = [optitrack.pose_position_x(j); optitrack.pose_position_y(j); optitrack.pose_orientation_yaw(j)]; 
            [x_k_k{l},P_k_k] = Kalman_update(P_k_k1,gamma,R2,x_k_k1{l},z,"optitrack",j,k);
            continue;
        case 3 % mouse
            k = k + 1; dt = mouse.time(k) - time(l-1); time(l) = mouse.time(k);
            [x_k_k1{l},P_k_k1,acc{l},gamma] = Kalman_predict(P_k_k,Q,x_k_k{l-1},i,imu,dt);
            z = [mouse.v_x(k); mouse.v_y(k)];
            [x_k_k{l},P_k_k] = Kalman_update(P_k_k1,gamma,R3,x_k_k1{l},z,"mouse",j,k);
            continue;
    end
    std{l} = (Standard_deviation(P_k_k))';
end

x_k_k = cell2mat(x_k_k); x_k_k1 = cell2mat(x_k_k1); acc = cell2mat(acc); std = cell2mat(std);
time = time - time(1); % Time start at 0
%% plotting
figure(); 

subplot(4,1,1); hold on; title("States")
h(1) = plot(time,x_k_k(1,:),'DisplayName','x','linewidth',1); h(2) = plot(time,x_k_k(4,:),'DisplayName','vx','linewidth',1); h(3) = plot(time,acc(1,:),'DisplayName','ax','linewidth',1); 
plot(time,x_k_k(1,:)-std(1,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(1,:)+std(1,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
plot(time,x_k_k(4,:)-std(4,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.'); plot(time,x_k_k(4,:)+std(4,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.');
legend(h);clear h;


subplot(4,1,2); hold on; 
h(1) = plot(time,x_k_k(2,:),'DisplayName','y'); h(2) = plot(time,x_k_k(5,:),'DisplayName','vy'); h(3) = plot(time,acc(2,:),'DisplayName','ay');
plot(time,x_k_k(2,:)-std(2,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(2,:)+std(2,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
plot(time,x_k_k(5,:)-std(5,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.'); plot(time,x_k_k(5,:)+std(5,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.');
legend(h);clear h;



subplot(4,1,3); hold on; 
h(1) = plot(time,x_k_k(3,:),'DisplayName','gamma'); h(2) = plot(time,x_k_k(6,:),'DisplayName','wz');
plot(time,x_k_k(3,:)-std(3,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(3,:)+std(3,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
plot(time,x_k_k(6,:)-std(6,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.'); plot(time,x_k_k(6,:)+std(6,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.');
legend(h);clear h;


subplot(4,1,4); hold on; 
h(1) = plot(time,x_k_k(7,:),'DisplayName','bx'); h(2) = plot(time,x_k_k(8,:),'DisplayName','by'); 
plot(time,x_k_k(7,:)-std(7,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(7,:)+std(7,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
plot(time,x_k_k(8,:)-std(8,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.'); plot(time,x_k_k(8,:)+std(8,:),'color',[0.8500, 0.3250, 0.0980],'LineStyle','-.');
legend(h);clear h;

%% Optitrack comparaison

optitrack_plot.time = optitrack_plot.time - imu.time(1); mouse.time = mouse.time -imu.time(1); imu.time = imu.time - imu.time(1);

figure();

subplot(6,1,1); hold on; title("States versus Optitrack")
h(1) = plot(time,x_k_k(1,:),'DisplayName','x_{kalman}','linewidth',1); h(2) = plot(optitrack_plot.time,optitrack_plot.pose_position_x,'DisplayName','x_{optitrack}','linewidth',1); 
plot(time,x_k_k(1,:)-std(1,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(1,:)+std(1,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
legend(h);clear h;


subplot(6,1,2); hold on;
h(1) = plot(time,x_k_k(2,:),'DisplayName','y_{kalman}','linewidth',1); h(2) = plot(optitrack_plot.time,optitrack_plot.pose_position_y,'DisplayName','y_{optitrack}','linewidth',1); 
plot(time,x_k_k(2,:)-std(2,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(2,:)+std(2,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
legend(h);clear h;


subplot(6,1,3); hold on
h(1) = plot(time,x_k_k(3,:),'DisplayName','\gamma_{kalman}','linewidth',1); h(2) = plot(optitrack_plot.time,optitrack_plot.pose_orientation_yaw,'DisplayName','\gamma_{optitrack}','linewidth',1); 
plot(time,x_k_k(3,:)-std(3,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(3,:)+std(3,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
legend(h);clear h;

optitrack_plot.vx = [0; diff(optitrack_plot.pose_position_x(:))./diff(optitrack_plot.time(:))];
optitrack_plot.vy = [0; diff(optitrack_plot.pose_position_y(:))./diff(optitrack_plot.time(:))];
optitrack_plot.wz = [0; diff(optitrack_plot.pose_orientation_yaw(:))./diff(optitrack_plot.time(:))];


subplot(6,1,4); hold on; 
h(1) = plot(time,x_k_k(4,:),'DisplayName','vx_{kalman}'); h(2) = plot(optitrack_plot.time,optitrack_plot.vx,'DisplayName','vx_{optitrack}'); h(3) = plot(mouse.time,mouse.v_x,'DisplayName','vx_{mouse}'); 
plot(time,x_k_k(4,:)-std(4,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(4,:)+std(4,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.');
ylim([-1.5 1.5]);
legend(h);clear h;

subplot(6,1,5); hold on; 
h(1) = plot(time,x_k_k(5,:),'DisplayName','vy_{kalman}'); h(2) = plot(optitrack_plot.time,optitrack_plot.vy,'DisplayName','vy_{optitrack}'); h(3) = plot(mouse.time,mouse.v_y,'DisplayName','vy_{mouse}');
plot(time,x_k_k(5,:)-std(5,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(5,:)+std(5,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
legend(h);clear h;

subplot(6,1,6); hold on; 
h(1) = plot(time,x_k_k(6,:),'DisplayName','wz_{kalman}'); h(2) = plot(optitrack_plot.time,optitrack_plot.wz,'DisplayName','wz_{optitrack}'); h(3) = plot(imu.time,imu.gyro_z,'DisplayName','wz_{imu}');
plot(time,x_k_k(6,:)-std(6,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); plot(time,x_k_k(6,:)+std(6,:),'color',[0, 0.4470, 0.7410],'LineStyle','-.'); 
ylim([-4,4]); % Hide noisy value
legend(h);clear h;

%% Functions

function [x_k_k1,P_k_k1,acc,gamma] = Kalman_predict(P,Q,x,i,imu,h)
    [vx,vy,wz,ax,ay,bx,by,gamma] = Update_var(x,i,imu);

    MatF1 = [-sin(gamma), -cos(gamma);
             cos(gamma), -sin(gamma)];

    vect = [ax-bx; ay-by];

    MatF2 = [-cos(gamma), sin(gamma);
             -sin(gamma), -cos(gamma)];

    MatR = [cos(gamma), -sin(gamma);
            sin(gamma), cos(gamma)];

    F = [zeros(3), eye(3), zeros(3,2);
         zeros(2), MatF1 * vect, zeros(2,3), MatF2;
         zeros(3,8)];

    acc = MatR * [ax-bx;
                  ay-by];
    x_dot = [vx;
             vy;
             wz;
             MatR * [ax-bx;
                     ay-by];
             0;
             0;
             0];

    x_k_k1 = h * x_dot + x;
    x_k_k1(3) = wrapToPi(x_k_k1(3));
    
    P_k_k1 = h * (F * P + P * F' + Q) + P;
end

function [x_k_k,P_k_k] = Kalman_update(P_k_k1,gamma,R,x_k_k1,z,type,j,k)
    MatR = [cos(gamma), -sin(gamma);
            sin(gamma), cos(gamma)];
    std = Standard_deviation(P_k_k1);

    if(type == "mouse")
        z_mouse_check = MatR * z;
    end

    switch(type)
        case "imu"
            H = [0 0 0 0 0 1 0 0];
        case "optitrack"
            if((z(1) < x_k_k1(1) - 5 * std(1)) || (z(1) > x_k_k1(1) + 5 * std(1))|| ...
               (z(2) < x_k_k1(2) - 5 * std(2)) || (z(2) > x_k_k1(2) + 5 * std(2))|| ...
               (z(3) < x_k_k1(3) - 5 * std(3)) || (z(3) > x_k_k1(3) + 5 * std(3)))
                x_k_k = x_k_k1;
                P_k_k = P_k_k1;
                disp("ignored optitrack measurement " + j)
                return
            end
            H = [eye(3), zeros(3,5)];
        case "mouse"
            if((z_mouse_check(1) < x_k_k1(4) - 3 * std(4)) || (z_mouse_check(1) > x_k_k1(4) + 3 * std(4))|| ...
               (z_mouse_check(2) < x_k_k1(5) - 3 * std(5)) || (z_mouse_check(2) > x_k_k1(5) + 3 * std(5)))
                x_k_k = x_k_k1;
                P_k_k = P_k_k1;
                disp("ignored mouse measurement " + k)
                return
            end
            vect = [sin(gamma) * z(1) - cos(gamma) * z(2);
                    cos(gamma) * z(1) - sin(gamma) * z(2)];
            H = [zeros(2),vect , MatR,zeros(2,3)];
    end

    
    K = P_k_k1 * H' / (H * P_k_k1 * H' + R);
    y = z- H*x_k_k1;

    if(type == "optitrack") 
        y(3) = wrapToPi(y(3));
    end

    x_k_k = x_k_k1 + K * y;
    x_k_k(3) = wrapToPi(x_k_k(3));
    P_k_k = (eye(size(P_k_k1,1)) - K * H) * P_k_k1;
end

function [vx,vy,wz,ax,ay,bx,by,gamma] = Update_var(x,i,imu)
    vx = x(4);
    vy = x(5);
    wz = x(6);
    bx = x(7);
    by = x(8);
    ax = -imu.acc_x(i);
    ay = -imu.acc_y(i);
    gamma = x(3);
end

function std = Standard_deviation(P_k_k)
    for i = 1:size(P_k_k,1)
        std(i) = sqrt(P_k_k(i,i));
    end
end