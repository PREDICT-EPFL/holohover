% Plot time and closed-loop


%close all;
clear all;
clc;

d = dir;
Nagents = 1;

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
    

end



%% closed-loop trajectories


figure()
subplot(3,1,1);
for i = 1:Nagents
    stairs(uu{i}(1,:) - uu_bc{i}(1,:));
    hold on
end

subplot(3,1,2);
for i = 1:Nagents
    stairs(uu{i}(2,:) - uu_bc{i}(2,:));
    hold on
end

subplot(3,1,3);
for i = 1:Nagents
    stairs(uu{i}(3,:) - uu_bc{i}(3,:));
    hold on
end

%% detailed debugging

for i = 1:Nagents

figure()
subplot(3,3,1);
stairs(xx{i}(1,:));
ylabel("x");

subplot(3,3,2);
stairs(xx{i}(2,:));
ylabel("y");

subplot(3,3,3);
stairs(xx{i}(5,:));
ylabel("yaw");

subplot(3,3,4);
stairs(xx{i}(3,:));
ylabel("vx");

subplot(3,3,5);
stairs(xx{i}(4,:));
ylabel("vy");

subplot(3,3,6);
stairs(xx{i}(6,:));
ylabel("w");

subplot(3,3,7);
stairs(uu{i}(1,:), 'DisplayName',"after conversion");
hold on
stairs(uu_bc{i}(1,:), 'DisplayName',"before conversion");
ylabel("ax");
legend();

subplot(3,3,8);
stairs(uu{i}(2,:), 'DisplayName',"after conversion");
hold on
stairs(uu_bc{i}(2,:), 'DisplayName',"before conversion");
ylabel("ay");
legend();


subplot(3,3,9);
stairs(uu{i}(3,:), 'DisplayName',"after conversion");
hold on
stairs(uu_bc{i}(3,:), 'DisplayName',"before conversion");
ylabel("aw");
legend();
title(sprintf("Agent %i ",i));

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










