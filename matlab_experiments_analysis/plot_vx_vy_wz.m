function [ ] = plot_vx_vy_wz(ref,state,name,max_value)
    figure()
    subplot(1,3,1)
    hold on
    plot((ref.time-state.time(1))/1e9,ref.vx,'r-','LineWidth',1.2)
    plot((state.time-state.time(1))/1e9,state.vx,'b-','LineWidth',1.2)
    plot([0 (state.time(end)-state.time(1))/1e9],[max_value(1) max_value(1)],'m-','LineWidth',1)
    plot([0 (state.time(end)-state.time(1))/1e9],[-max_value(1) -max_value(1)],'m-','LineWidth',1)
    %axis([0 3.5e9 0 0.25])
    legend('Reference', name+' result','Upper bound','Lower bound','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('$v_x$ ($m.s^{-1}$)','FontSize',15,'interpreter', 'latex')
    grid on

    subplot(1,3,2)
    hold on
    plot((state.time-state.time(1))/1e9,state.vy,'b-','LineWidth',1.2)
    plot((ref.time-state.time(1))/1e9,ref.vy,'r-','LineWidth',1.2)
    plot([0 (state.time(end)-state.time(1))/1e9],[max_value(2) max_value(2)],'m-','LineWidth',1)
    plot([0 (state.time(end)-state.time(1))/1e9],[-max_value(2) -max_value(2)],'m-','LineWidth',1)
    legend('Reference', name+' result','Upper bound','Lower bound','FontSize',11,'interpreter', 'latex')
    %axis([0 3.5e9 0 0.25])
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('$v_y$ ($m.s^{-1}$)','FontSize',15,'interpreter', 'latex')
    grid on

    subplot(1,3,3)
    hold on
    plot((state.time-state.time(1))/1e9,state.wz,'b-','LineWidth',1.2)
    plot((ref.time-state.time(1))/1e9,ref.wz,'r-','LineWidth',1.2)
    plot([0 (state.time(end)-state.time(1))/1e9],[max_value(3) max_value(3)],'m-','LineWidth',1)
    plot([0 (state.time(end)-state.time(1))/1e9],[-max_value(3) -max_value(3)],'m-','LineWidth',1)
    %axis([0 3.5e9 -0.1 0.1])
    legend('Reference', name+' result','Upper bound','Lower bound','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('$w_z$ ($rad.s^{-1}$)','FontSize',15,'interpreter', 'latex')
    grid on
    
end

