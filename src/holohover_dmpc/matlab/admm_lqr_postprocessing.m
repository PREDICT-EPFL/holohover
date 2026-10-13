% Plot time and closed-loop


%close all;
clear all;
clc;

d = dir;
Nagents = 1;
dt = 0.050;

for i = 1:Nagents
    str = sprintf("dmpc_lqr_log_agent%i*",i-1);
    file{i} = dir(str);
    t_lqr{i} = readtable(file{i}.name);
end
LQR_steps = size(t_lqr{1},1);
for i = 2:Nagents
    if size(t_lqr{i},1) < LQR_steps
        LQR_steps = size(t_lqr{i},1);
    end
end


for i = 1:Nagents
    uu_dmpc{i} = -ones(6,LQR_steps);
    uu_lqr{i} = -ones(6,LQR_steps);
    uu_bc{i} = -ones(3,LQR_steps);
    uu{i} = -ones(3,LQR_steps);
end

for i = 1:Nagents    
    xx{i}(1,:)        = t_lqr{i}.x0_1_.';
    xx{i}(2,:)        = t_lqr{i}.x0_2_.';
    xx{i}(3,:)        = t_lqr{i}.x0_3_.';
    xx{i}(4,:)        = t_lqr{i}.x0_4_.';
    xx{i}(5,:)        = t_lqr{i}.x0_5_.';
    xx{i}(6,:)        = t_lqr{i}.x0_6_.';
    uu_dmpc{i}(1,:)        = t_lqr{i}.u_1_dmpc_(1:LQR_steps).';
    uu_dmpc{i}(2,:)        = t_lqr{i}.u_2_dmpc_(1:LQR_steps).';
    uu_dmpc{i}(3,:)        = t_lqr{i}.u_3_dmpc_(1:LQR_steps).';
    uu_lqr{i}(1,:)        = t_lqr{i}.u_1_lqr_(1:LQR_steps).';
    uu_lqr{i}(2,:)        = t_lqr{i}.u_2_lqr_(1:LQR_steps).';
    uu_lqr{i}(3,:)        = t_lqr{i}.u_3_lqr_(1:LQR_steps).';
    uu_bc{i}(1,:) = t_lqr{i}.u_1_bc_(1:LQR_steps).';
    uu_bc{i}(2,:) = t_lqr{i}.u_2_bc_(1:LQR_steps).';
    uu_bc{i}(3,:) = t_lqr{i}.u_3_bc_(1:LQR_steps).';
    uu{i}(1,:)      = t_lqr{i}.u_1_acc_(1:LQR_steps).';
    uu{i}(2,:)      = t_lqr{i}.u_2_acc_(1:LQR_steps).';
    uu{i}(3,:)      = t_lqr{i}.u_3_acc_(1:LQR_steps).';
    try
        xx_dmpc{i}(1,:) = t_lqr{i}.x_1_dmpc_(1:LQR_steps).';
        xx_dmpc{i}(2,:) = t_lqr{i}.x_2_dmpc_(1:LQR_steps).';
        xx_dmpc{i}(3,:) = t_lqr{i}.x_3_dmpc_(1:LQR_steps).';
        xx_dmpc{i}(4,:) = t_lqr{i}.x_4_dmpc_(1:LQR_steps).';
        xx_dmpc{i}(5,:) = t_lqr{i}.x_5_dmpc_(1:LQR_steps).';
        xx_dmpc{i}(6,:) = t_lqr{i}.x_6_dmpc_(1:LQR_steps).';
        xx_lqr{i}(1,:) = t_lqr{i}.x_1_lqr_(1:LQR_steps).';
        xx_lqr{i}(2,:) = t_lqr{i}.x_2_lqr_(1:LQR_steps).';
        xx_lqr{i}(3,:) = t_lqr{i}.x_3_lqr_(1:LQR_steps).';
        xx_lqr{i}(4,:) = t_lqr{i}.x_4_lqr_(1:LQR_steps).';
        xx_lqr{i}(5,:) = t_lqr{i}.x_5_lqr_(1:LQR_steps).';
        xx_lqr{i}(6,:) = t_lqr{i}.x_6_lqr_(1:LQR_steps).';
    catch
    end
    try
        xxd{i}(1,:)       = t_lqr{i}.xd_1_.';
        xxd{i}(2,:)       = t_lqr{i}.xd_2_.';
        xxd{i}(3,:)       = t_lqr{i}.xd_5_.';
    catch
    end
end



%% closed-loop trajectories
t = 0:dt:(LQR_steps-1)*dt;

figure()
subplot(3,1,1);
for i = 1:Nagents
    stairs(t,uu{i}(1,:) - uu_bc{i}(1,:));
    hold on
end
grid on

subplot(3,1,2);
for i = 1:Nagents
    stairs(t,uu{i}(2,:) - uu_bc{i}(2,:));
    hold on
end
grid on

subplot(3,1,3);
for i = 1:Nagents
    stairs(t,uu{i}(3,:) - uu_bc{i}(3,:));
    hold on
end
grid on
%% detailed debugging

for i = 1:Nagents

figure()
subplot(3,3,1);
stairs(t,xx{i}(1,:),'DisplayName','actual');
hold on
ylabel("x");
try
    stairs(t,xxd{i}(1,:),'DisplayName','setpoint');
catch
end
try
stairs(t,xx_dmpc{i}(1,:),'DisplayName','DMPC-pred');
stairs(xx_lqr{i}(1,:),'DisplayName','LQR-pred');
catch
end
legend();
grid on
title(file{1}.folder(end-20:end));

subplot(3,3,2);
stairs(t,xx{i}(2,:),'DisplayName','actual');
ylabel("y");
hold on
try
    stairs(t,xxd{i}(2,:),'DisplayName','setpoint');
catch
end
try
stairs(t,xx_dmpc{i}(2,:),'DisplayName','DMPC-pred');
stairs(t,xx_lqr{i}(2,:),'DisplayName','LQR-pred');
catch
end
legend();
grid on

subplot(3,3,3);
stairs(t,xx{i}(5,:),'DisplayName','actual');
ylabel("yaw");
hold on
try
    stairs(t,xxd{i}(3,:),'DisplayName','setpoint');
catch
end
try
stairs(t,xx_dmpc{i}(5,:),'DisplayName','DMPC-pred');
stairs(t,xx_lqr{i}(5,:),'DisplayName','LQR-pred');
catch
end
legend();
grid on

subplot(3,3,4);
stairs(t,xx{i}(3,:),'DisplayName','actual');
ylabel("vx");
hold on
try
stairs(t,xx_dmpc{i}(3,:),'DisplayName','DMPC-pred');
stairs(t,xx_lqr{i}(3,:),'DisplayName','LQR-pred');
catch
end
legend();
grid on

subplot(3,3,5);
stairs(t,xx{i}(4,:),'DisplayName','actual');
ylabel("vy");
hold on
try
stairs(t,xx_dmpc{i}(4,:),'DisplayName','DMPC-pred');
stairs(t,xx_lqr{i}(4,:),'DisplayName','LQR-pred');
catch
end
legend();
grid on

subplot(3,3,6);
stairs(t,xx{i}(6,:),'DisplayName','actual');
ylabel("w");
hold on
try
stairs(t,xx_dmpc{i}(6,:),'DisplayName','DMPC-pred');
stairs(t,xx_lqr{i}(6,:),'DisplayName','LQR-pred');
catch
end
legend();
grid on

subplot(3,3,7);
stairs(t,uu{i}(1,:), 'DisplayName',"after conversion");
hold on
stairs(t,uu_bc{i}(1,:), 'DisplayName',"before conversion");
ylabel("ax");
legend();
grid on

subplot(3,3,8);
stairs(t,uu{i}(2,:), 'DisplayName',"after conversion");
hold on
stairs(t,uu_bc{i}(2,:), 'DisplayName',"before conversion");
ylabel("ay");
legend();
grid on

subplot(3,3,9);
stairs(t,uu{i}(3,:), 'DisplayName',"after conversion");
hold on
stairs(t,uu_bc{i}(3,:), 'DisplayName',"before conversion");
ylabel("aw");
legend();
title(sprintf("Agent %i ",i));
grid on
% figure()
% subplot(3,1,1);
% stairs(uu{i}(1,:));
% hold on
% stairs(uu_bc{i}(1,:));
% 
% subplot(3,1,2);
% stairs(uu{i}(2,:));
% hold on
% stairs(uu_bc{i}(2,:));
% 
% subplot(3,1,3);
% stairs(uu{i}(3,:));
% hold on
% stairs(uu_bc{i}(3,:));


end










