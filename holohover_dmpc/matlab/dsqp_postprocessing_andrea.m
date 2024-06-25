% Plot time and closed-loop


%close all;
clear all;
clc;


[stat, dir_name] = system("ls ~/holohover-docker/log -rt | grep remote | tail -n1");

str = sprintf("cp ~/holohover-docker/log/%s/*/log/dmpc* ./tmp",strtrim(dir_name));
system(str);


d = dir;
Nagents = 4;
Ndsqp = 1; %dsqp iterations per MPC step
Nadmm = 6; %admm iterations per sqp iteration
dt = 0.100; %sampling time

for i = 1:Nagents
    str = sprintf("tmp/dmpc_time_measurement_agent%i*",i-1);
    name = dir(str);
    filename = sprintf("tmp/%s",name.name);
    t_admm{i} = readtable(filename);
    rows(i) = size(t_admm{i},1);
end


system("rm ./tmp/*");


rows_ = min(rows);
rows = 1:rows_;
for i = 1:Nagents
    t_admm{i} = t_admm{i}(rows,:);
    if Nadmm > 1
        t_dsqp{i} = t_admm{i}(mod(rows,Nadmm)==1,1:32);
    else
        t_dsqp{i} = t_admm{i}(:,1:32);
    end
    rows_dsqp = 1:size(t_dsqp{i},1);
    if Ndsqp > 1
        t_mpc{i} = t_dsqp{i}(mod(rows_dsqp,Ndsqp)==1,1:27);
    else
        t_mpc{i} = t_dsqp{i}(:,1:27);
    end
end
ADMM_iter = size(t_admm{1},1);
MPC_steps = size(t_mpc{1},1);
DSQP_iter = size(t_dsqp{1},1);

SqpTime         = -ones(MPC_steps,Nagents);
%AdmmTime        = -ones(MPC_steps,Nagents);
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
    DsqpTime(:,i)     = t_mpc{i}.dsqp_time_us_(1:MPC_steps)/1000;
    %AdmmTime(:,i)     = t_mpc{i}.admm_time_us_/1000;
    %AdmmIterTime(:,i) = t_admm{i}.admm_iter_time_us_/1000;
    build_qpTime(:,i) = t_dsqp{i}.build_qp_time_us_(1:DSQP_iter)/1000;
    Loc_qpTime(:,i)   = t_admm{i}.loc_qp_time_us_(1:ADMM_iter)/1000;
    ZcommTime(:,i)= t_admm{i}.zcomm_time_us_(1:ADMM_iter)/1000;
    ZbarcommTime(:,i) = t_admm{i}.zbarcomm_time_us_(1:ADMM_iter)/1000;
    Zasync(:,i) = t_admm{i}.z_is_async(1:ADMM_iter);
    Zbarasync(:,i) = t_admm{i}.zbar_is_async(1:ADMM_iter);

    xx{i}(1,:)        = t_mpc{i}.x0_1_(1:MPC_steps).';
    xx{i}(2,:)        = t_mpc{i}.x0_2_(1:MPC_steps).';
    xx{i}(3,:)        = t_mpc{i}.x0_3_(1:MPC_steps).';
    xx{i}(4,:)        = t_mpc{i}.x0_4_(1:MPC_steps).';
    xx{i}(5,:)        = t_mpc{i}.x0_5_(1:MPC_steps).';
    xx{i}(6,:)        = t_mpc{i}.x0_6_(1:MPC_steps).';
    uu{i}(1,:)        = t_mpc{i}.u0_1_(1:MPC_steps).';
    uu{i}(2,:)        = t_mpc{i}.u0_2_(1:MPC_steps).';
    uu{i}(3,:)        = t_mpc{i}.u0_3_(1:MPC_steps).';
    try
        uu_bc{i}(1,:)      = t_mpc{i}.u0bc_1_(1:MPC_steps).';
        uu_bc{i}(2,:)      = t_mpc{i}.u0bc_2_(1:MPC_steps).';
        uu_bc{i}(3,:)      = t_mpc{i}.u0bc_3_(1:MPC_steps).';
    catch
    end
    xxd{i}(1,:)       = t_mpc{i}.xd_1_(1:MPC_steps).';
    xxd{i}(2,:)       = t_mpc{i}.xd_2_(1:MPC_steps).';
    xxd{i}(3,:)       = t_mpc{i}.xd_5_(1:MPC_steps).';

end

for i = 2:Nagents
    xxd{i} = xxd{i} + xxd{i-1};
end

dsqpTime = max(DsqpTime,[],2);
%admmTime        = max(AdmmTime,[],2);
%admmIterTime    = max(AdmmIterTime,[],2);
loc_qpTime      = max(Loc_qpTime,[],2);
zcommTime   = max(ZcommTime,[],2);
zbarcommTime= max(ZbarcommTime,[],2);

rows = 1:size(ZcommTime,1);
for i = 1:Nagents
    for k = 1:Nadmm
        if k < Nadmm
            %admmIterTimeIter{i}{k} = AdmmIterTime(mod(rows,Nadmm)==k,i);
            loc_qpTimeIter{i}{k} = Loc_qpTime(mod(rows,Nadmm)==k,i);
            zcommTimeIter{i}{k} = ZcommTime(mod(rows,Nadmm)==k,i);
            zbarcommTimeIter{i}{k} = ZbarcommTime(mod(rows,Nadmm)==k,i);
            zasyncIter{i}{k} = Zasync(mod(rows,Nadmm)==k,i);
            zbarasyncIter{i}{k} = Zbarasync(mod(rows,Nadmm)==k,i);
        else
            %admmIterTimeIter{i}{k} = AdmmIterTime(mod(rows,Nadmm)==0,i);
            loc_qpTimeIter{i}{k} = Loc_qpTime(mod(rows,Nadmm)==0,i);
            zcommTimeIter{i}{k} = ZcommTime(mod(rows,Nadmm)==0,i);
            zbarcommTimeIter{i}{k} = ZbarcommTime(mod(rows,Nadmm)==0,i);
            zasyncIter{i}{k} = Zasync(mod(rows,Nadmm)==0,i);
            zbarasyncIter{i}{k} = Zbarasync(mod(rows,Nadmm)==0,i);
        end
    end
end


for i = 1:MPC_steps
    loc_qpTime_MPC_step(i) = sum( loc_qpTime( (i-1)*Nadmm*Ndsqp+1 : i*Nadmm*Ndsqp ) );
    zcommTime_MPC_step(i) = sum( zcommTime( (i-1)*Nadmm*Ndsqp+1 : i*Nadmm*Ndsqp ) );
    zbarcommTime_MPC_step(i) = sum( zbarcommTime( (i-1)*Nadmm*Ndsqp+1 : i*Nadmm*Ndsqp ) );
    for j = 1:Nagents
        Loc_qpTime_MPC_step(i,j) = sum( Loc_qpTime( (i-1)*Nadmm*Ndsqp+1 : i*Nadmm*Ndsqp , j ));
        ZcommTime_MPC_step(i,j) = sum( ZcommTime( (i-1)*Nadmm*Ndsqp+1 : i*Nadmm*Ndsqp , j ) );
        ZbarcommTime_MPC_step(i,j) = sum( ZbarcommTime( (i-1)*Nadmm*Ndsqp+1 : i*Nadmm*Ndsqp , j ) );
        Build_qpTime_MPC_step(i,j) = sum( build_qpTime( (i-1)*Ndsqp+1 : i*Ndsqp , j ));
        MPC_step_time(i,j) = ( t_mpc{j}.convert_uacc_time_us_(i) + t_mpc{j}.publish_signal_time_us_(i) + t_mpc{j}.update_setpoint_time_us_(i) + t_mpc{j}.dsqp_time_us_(i) ) / 1000;
    end
end

%% closed-loop trajectories

t = 0:dt:(MPC_steps-1)*dt;
lw = 2;

figure('units','normalized','outerposition',[0 0 1 1])
subplot(6,3,1);
for i = 1:Nagents
    plot(t,xx{i}(1,:),'LineWidth',lw);
    hold on
    plot(t,xxd{i}(1,:),'-k','LineWidth',1.2);
end
grid on
ylabel("x");
ylim([-1.2,1.2]);
legend();

subplot(6,3,2);
for i = 1:Nagents
    plot(t,xx{i}(2,:),'LineWidth',lw);
    hold on
    plot(t,xxd{i}(2,:),'-k','LineWidth',1.2);
    hold on
end
grid on
ylim([-0.7,0.7]);
ylabel("y");
legend();

subplot(6,3,3);
for i = 1:Nagents
    plot(t,xx{i}(5,:),'LineWidth',lw);
    hold on
end
grid on
ylabel("yaw");
legend();

subplot(6,3,4);
for i = 1:Nagents
    plot(t,xx{i}(3,:),'LineWidth',lw);
    hold on
end
ylabel("vx");
grid on
legend();

subplot(6,3,5);
for i = 1:Nagents
    plot(t,xx{i}(4,:),'LineWidth',lw);
    hold on
end
ylabel("vy");
grid on
legend();

subplot(6,3,6);
for i = 1:Nagents
    plot(t,xx{i}(6,:),'LineWidth',lw);
    hold on
end
ylabel("w");
grid on
legend();

subplot(6,3,7);
for i = 1:Nagents
    stairs(t,uu{i}(1,:),'LineWidth',lw);
    hold on
end
ylabel("ax");
grid on
legend();

subplot(6,3,8);
for i = 1:Nagents
    plot(t,uu{i}(2,:),'LineWidth',lw);
    hold on
end
ylabel("ay");
grid on
legend();

subplot(6,3,9);
for i = 1:Nagents
    plot(t,uu{i}(3,:),'LineWidth',lw);
    hold on
end
ylabel("aw");
grid on
legend();

subplot(6,3,10);
for i = 1:Nagents
    stairs(t,DsqpTime(:,i));
    hold on
end
ylabel("Solve OCP [ms]");
legend();

subplot(6,3,11);
for i = 1:Nagents
    stairs(t,Loc_qpTime_MPC_step(:,i));
    hold on
end
ylabel("Local QP [ms]");
legend();

subplot(6,3,12);
for i = 1:Nagents
    stairs(t,ZcommTime_MPC_step(:,i));
    hold on
end
ylabel("z c. [ms]");
legend();

subplot(6,3,13);
for i = 1:Nagents
    stairs(t,ZbarcommTime_MPC_step(:,i));
    hold on
end
ylabel("zbar c. [ms]");
legend();


subplot(6,3,14);
for i = 1:Nagents-1
    for k = 1:size(xx{i},2)
        distance(i,k) = sqrt( (xx{i}(1,k) - xx{i+1}(1,k))^2 + (xx{i}(2,k) - xx{i+1}(2,k))^2);
    end
    stairs(t,distance(i,:));
    hold on;
end
ylabel("distance [m]");
legend();

subplot(6,3,15);
for i = 1:Nagents
    stairs(t,Build_qpTime_MPC_step(:,i));
    hold on;
end
ylabel("build QP [ms]");
legend();

subplot(6,3,16);
for i = 1:Nagents
    stairs(t,MPC_step_time(:,i));
    hold on
end
ylabel("MPC step time [ms]");


% try
% figure()
% subplot(3,1,1);
% for i = 1:Nagents
%     stairs(t,uu_bc{i}(1,:) - uu_bc{i}(1,:));
%     hold on
% end
% 
% subplot(3,1,2);
% for i = 1:Nagents
%     stairs(t,uu{i}(2,:) - uu_bc{i}(2,:));
%     hold on
% end
% 
% subplot(3,1,3);
% for i = 1:Nagents
%     stairs(t,uu{i}(3,:) - uu_bc{i}(3,:));
%     hold on
% end
% catch
% end

%%
% try
% figure()
% subplot(3,1,1);
% stairs(t,uu{i}(1,:));
% hold on
% stairs(t,uu_bc{i}(1,:));
% 
% subplot(3,1,2);
% stairs(t,uu{i}(2,:));
% hold on
% stairs(t,uu_bc{i}(2,:));
% 
% subplot(3,1,3);
% stairs(t,uu{i}(3,:));
% hold on
% stairs(t,uu_bc{i}(3,:));
% 
% catch
% end

%% detailed debugging

% for i = 1:Nagents
% 
% figure()
% subplot(3,3,1);
% stairs(t,xx{i}(1,:));
% ylabel("x");
% 
% subplot(3,3,2);
% stairs(t,xx{i}(2,:));
% ylabel("y");
% 
% subplot(3,3,3);
% stairs(t,xx{i}(5,:));
% ylabel("yaw");
% 
% subplot(3,3,4);
% stairs(t,xx{i}(3,:));
% ylabel("vx");
% 
% subplot(3,3,5);
% stairs(t,xx{i}(4,:));
% ylabel("vy");
% 
% subplot(3,3,6);
% stairs(t,xx{i}(6,:));
% ylabel("w");
% 
% subplot(3,3,7);
% stairs(t,uu{i}(1,:),"DisplayName","after conversion");
% hold on
% try
% stairs(t,uu_bc{i}(1,:),"DisplayName","before conversion");
% catch
% end
% ylabel("ax");
% legend();
% 
% subplot(3,3,8);
% stairs(t,uu{i}(2,:),"DisplayName","after conversion");
% hold on
% try
% stairs(t,uu_bc{i}(2,:),"DisplayName","before conversion");
% catch
% end
% ylabel("ay");
% legend();
% 
% subplot(3,3,9);
% stairs(t,uu{i}(3,:),"DisplayName","after conversion");
% hold on
% try
% stairs(t,uu_bc{i}(3,:),"DisplayName","before conversion");
% catch
% end
% ylabel("aw");
% legend();
% 
% end

%% detailed timing statistics in one figure
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:Nagents
for k = 1:Nadmm
    subplot(5,Nadmm,k); stairs(loc_qpTimeIter{i}{k}); str = sprintf('ADMM Iter %i',k); ylabel("loc QP"); xlabel("dsqp iter"); title(str); legend(); hold on;
    subplot(5,Nadmm,Nadmm+k); stairs(zcommTimeIter{i}{k}); str = sprintf('ADMM Iter %i',k); ylabel("z comm."); xlabel("dsqp iter"); title(str); legend(); hold on;
    subplot(5,Nadmm,2*Nadmm+k); stairs(zbarcommTimeIter{i}{k}); str = sprintf('ADMM Iter %i',k); ylabel("zbar comm."); xlabel("dsqp iter"); title(str); legend(); hold on;
    subplot(5,Nadmm,3*Nadmm+k); stairs(zasyncIter{i}{k}); str = sprintf('ADMM Iter %i',k); ylabel("z is async"); xlabel("dsqp iter"); title(str); legend(); hold on;
    subplot(5,Nadmm,4*Nadmm+k); stairs(zbarasyncIter{i}{k}); str = sprintf('ADMM Iter %i',k); ylabel("zbar is async"); xlabel("dsqp iter"); title(str); legend(); hold on;
end
end

%%


figure();
for i = 1:Nagents
subplot(3,1,1);
plot(Loc_qpTime(:,i));
hold on
ylabel("loc solve");

subplot(3,1,2);
plot(ZcommTime(:,i));
hold on
ylabel("z comm.");

subplot(3,1,3);
plot(ZbarcommTime(:,i));
ylabel("zbar comm.")
hold on


end

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

% figure();
% for i = 1:Nagents    
%     subplot(Nagents,5,(i-1)*5+1)
%     histogram(t_mpc{i}.dsqp_time_us_/1000);
%     xlabel('dSQP time to solve OCP / ms','Interpreter','latex');
%     
%     subplot(Nagents,5,(i-1)*5+2)
%     histogram(t_dsqp{i}.dsqp_iter_time_us_/1000);
%     xlabel('dSQP Iteration time / ms','Interpreter','latex');
%     
%     subplot(Nagents,5,(i-1)*5+3)
%     histogram(t_admm{i}.loc_qp_time_us_/1000);
%     xlabel('Local QP time / ms','Interpreter','latex');
%     
%     subplot(Nagents,5,(i-1)*5+4)
%     histogram(t_admm{i}.zcomm_time_us_/1000);
%     xlabel('z communication time / ms','Interpreter','latex');
% 
%     subplot(Nagents,5,(i-1)*5+5)
%     histogram(t_admm{i}.zbarcomm_time_us_/1000);
%     xlabel('zbar communication time / ms','Interpreter','latex');
% end







