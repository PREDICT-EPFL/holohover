% Plot time and closed-loop


close all;
clear all;
clc;

d = dir;
Nagents = 4;
Nadmm = 5; %admm iterations per MPC step

for i = 1:Nagents
    str = sprintf("dmpc_time_measurement_agent%i*",i-1);
    file{i} = dir(str);
    t_admm{i} = readtable(file{i}.name);
    rows = 1:size(t_admm{i},1);
    t_mpc{i} = t_admm{i}(mod(rows,Nadmm)==1,1:17);
end
ADMM_iter = size(t_admm{1},1);
MPC_steps = size(t_mpc{1},1);

AdmmTime        = -ones(MPC_steps,Nagents);
AdmmIterTime    = -ones(ADMM_iter,Nagents);
Loc_qpTime      = -ones(ADMM_iter,Nagents);
ZcommTime       = -ones(ADMM_iter,Nagents);
ZbarcommTime    = -ones(ADMM_iter,Nagents);
for i = 1:Nagents
    xx{i} = -ones(6,MPC_steps);
    uu{i} = -ones(3,MPC_steps);
    xxd{i} = -ones(3,MPC_steps);
end

for i = 1:Nagents
    AdmmTime(:,i)     = t_mpc{i}.admm_time_us_/1000;
    AdmmIterTime(:,i) = t_admm{i}.admm_iter_time_us_/1000;
    Loc_qpTime(:,i)   = t_admm{i}.loc_qp_time_us_/1000;
    ZcommTime(:,i)= t_admm{i}.zcomm_time_us_/1000;
    ZbarcommTime(:,i) = t_admm{i}.zbarcomm_time_us_/1000;

    xx{i}(1,:)        = t_mpc{i}.x0_1_.';
    xx{i}(2,:)        = t_mpc{i}.x0_2_.';
    xx{i}(3,:)        = t_mpc{i}.x0_3_.';
    xx{i}(4,:)        = t_mpc{i}.x0_4_.';
    xx{i}(5,:)        = t_mpc{i}.x0_5_.';
    xx{i}(6,:)        = t_mpc{i}.x0_6_.';
    uu{i}(1,:)        = t_mpc{i}.u0_1_.';
    uu{i}(2,:)        = t_mpc{i}.u0_2_.';
    uu{i}(3,:)        = t_mpc{i}.u0_3_.';
    xxd{i}(1,:)        = t_mpc{i}.xd_1_.';
    xxd{i}(2,:)        = t_mpc{i}.xd_2_.';
    xxd{i}(3,:)        = t_mpc{i}.xd_3_.';

end

for i = 2:Nagents
    xxd{i} = xxd{i} + xxd{i-1};
end

admmTime        = max(AdmmTime,[],2);
admmIterTime    = max(AdmmIterTime,[],2);
loc_qpTime      = max(Loc_qpTime,[],2);
zcommTime   = max(ZcommTime,[],2);
zbarcommTime= max(ZbarcommTime,[],2);

rows = 1:size(ZcommTime,1);
for i = 1:Nagents
    for k = 1:Nadmm
        if k < Nadmm
            admmIterTimeIter{i}{k} = AdmmIterTime(mod(rows,Nadmm)==k,i);
            loc_qpTimeIter{i}{k} = Loc_qpTime(mod(rows,Nadmm)==k,i);
            zcommTimeIter{i}{k} = ZcommTime(mod(rows,Nadmm)==k,i);
            zbarcommTimeIter{i}{k} = ZbarcommTime(mod(rows,Nadmm)==k,i);

        else
            admmIterTimeIter{i}{k} = AdmmIterTime(mod(rows,Nadmm)==0,i);
            loc_qpTimeIter{i}{k} = Loc_qpTime(mod(rows,Nadmm)==0,i);
            zcommTimeIter{i}{k} = ZcommTime(mod(rows,Nadmm)==0,i);
            zbarcommTimeIter{i}{k} = ZbarcommTime(mod(rows,Nadmm)==0,i);
        end
    end
end

%% closed-loop trajectories

figure()
subplot(4,2,1);
for i = 1:Nagents
    plot(xx{i}(1,:));
    hold on
    plot(xxd{i}(1,:),'-k','LineWidth',1.2);
end
grid on
ylabel("x");

subplot(4,2,2);
for i = 1:Nagents
    plot(xx{i}(2,:));
    hold on
    plot(xxd{i}(2,:),'-k','LineWidth',1.2);
    hold on
end
grid on
ylim([-2,3]);
ylabel("y");

subplot(4,2,3);
for i = 1:Nagents
    plot(xx{i}(3,:));
    hold on
end
ylabel("vx");

subplot(4,2,4);
for i = 1:Nagents
    plot(xx{i}(4,:));
    hold on
end
ylabel("vy");

subplot(4,2,5);
for i = 1:Nagents
    stairs(uu{i}(1,:));
    hold on
end
ylabel("ax");

subplot(4,2,6);
for i = 1:Nagents
    plot(uu{i}(2,:));
    hold on
end
ylabel("ay");

subplot(4,2,7);
for i = 1:Nagents
    stairs(AdmmTime(:,i));
    hold on
end
ylabel("Solve OCP [ms]");


%% Detailed timing statistics

% figure('units','normalized','outerposition',[0 0 1 1])
% for i = 1:Nagents
%     subplot(Nagents,7,(i-1)*7+2); stairs(AdmmTime(:,i)); title('Solve OCP');
% end
% %saveas(gcf,'mpc_times_detailed.png','png')
% 
% figure('units','normalized','outerposition',[0 0 1 1])
% for i = 1:Nagents
%     for k = 1:Nadmm
%         subplot(Nagents,Nadmm,(i-1)*Nadmm+k); stairs(admmIterTimeIter{i}{k}); str = sprintf('ADMM Iter %i',k); title(str); str = sprintf('Agent %i',i); ylabel(str);
%     end
% end
% %saveas(gcf,'admm_iter_times_detailed.png','png')
% 
% figure('units','normalized','outerposition',[0 0 1 1])
% for i = 1:Nagents
%     for k = 1:Nadmm
%         subplot(Nagents,Nadmm,(i-1)*Nadmm+k); stairs(loc_qpTimeIter{i}{k}); str = sprintf('loc QP ADMM Iter %i',k); title(str); str = sprintf('Agent %i',i); ylabel(str);
%     end
% end
% %saveas(gcf,'loc_qp_times_detailed.png','png')
% 
% figure('units','normalized','outerposition',[0 0 1 1])
% for i = 1:Nagents
%     for k = 1:Nadmm
%         subplot(Nagents,Nadmm,(i-1)*Nadmm+k); stairs(zcommTimeIter{i}{k}); str = sprintf('zcomm ADMM Iter %i',k); title(str); str = sprintf('Agent %i',i); ylabel(str);
%     end
% end
% %saveas(gcf,'zcomm_times_detailed.png','png')
% 
% figure('units','normalized','outerposition',[0 0 1 1])
% for i = 1:Nagents
%     for k = 1:Nadmm
%         subplot(Nagents,Nadmm,(i-1)*Nadmm+k); stairs(zbarcommTimeIter{i}{k}); str = sprintf('zbarcomm ADMM Iter %i',k); title(str); str = sprintf('Agent %i',i); ylabel(str);
%     end
% end
% %saveas(gcf,'zbarcomm_times_detailed.png','png')

%% Histogram

figure();
for i = 1:Nagents    
    subplot(Nagents,5,(i-1)*5+1)
    histogram(t_mpc{i}.admm_time_us_/1000);
    xlabel('ADMM time to solve OCP / ms','Interpreter','latex');
    
    subplot(Nagents,5,(i-1)*5+2)
    histogram(t_admm{i}.admm_iter_time_us_/1000);
    xlabel('ADMM Iteration time / ms','Interpreter','latex');
    
    subplot(Nagents,5,(i-1)*5+3)
    histogram(t_admm{i}.loc_qp_time_us_/1000);
    xlabel('Local QP time / ms','Interpreter','latex');
    
    subplot(Nagents,5,(i-1)*5+4)
    histogram(t_admm{i}.zcomm_time_us_/1000);
    xlabel('z communication time / ms','Interpreter','latex');

    subplot(Nagents,5,(i-1)*5+5)
    histogram(t_admm{i}.zbarcomm_time_us_/1000);
    xlabel('zbar communication time / ms','Interpreter','latex');
end







