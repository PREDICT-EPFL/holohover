% Plot time and closed-loop


close all;
clear all;
clc;

d = dir;
Nagents = 4;

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
    uu_bc{i} = -ones(6,LQR_steps);
    uu{i} = -ones(3,LQR_steps);
end

for i = 1:Nagents    
    uu_dmpc{i}(1,:)        = t_lqr{i}.u_1_dmpc_(1:LQR_steps).';
    uu_dmpc{i}(2,:)        = t_lqr{i}.u_2_dmpc_(1:LQR_steps).';
    uu_dmpc{i}(3,:)        = t_lqr{i}.u_3_dmpc_(1:LQR_steps).';
    uu_lqr{i}(1,:)        = t_lqr{i}.u_1_lqr_(1:LQR_steps).';
    uu_lqr{i}(2,:)        = t_lqr{i}.u_2_lqr_(1:LQR_steps).';
    uu_lqr{i}(3,:)        = t_lqr{i}.u_3_lqr_(1:LQR_steps).';
    uu_bc{i} = uu_dmpc{i} + uu_lqr{i};
    uu{i}(1,:)      = t_lqr{i}.u_1_acc_(1:LQR_steps).';
    uu{i}(2,:)      = t_lqr{i}.u_2_acc(1:LQR_steps).';
    uu{i}(3,:)      = t_lqr{i}.u_3_acc(1:LQR_steps).';
    

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








