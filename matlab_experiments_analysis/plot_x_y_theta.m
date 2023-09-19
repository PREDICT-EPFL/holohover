function [ ] = plot_x_y_theta(ref,state,name,max_value)
    figure()
    subplot(1,3,1)
    hold on
    plot((ref.time-state.time(1))/1e9,ref.x,'r-','LineWidth',1.2)
    plot((state.time-state.time(1))/1e9,state.x,'b-','LineWidth',1.2)
    %axis([0 (state.time(end)-state.time(1))/1.5e9 -0.05 0.05])
    legend('Reference', name+' result','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('x (m)','FontSize',15,'interpreter', 'latex')
    grid on

    subplot(1,3,2)
    hold on
    plot((state.time-state.time(1))/1e9,state.y,'b-','LineWidth',1.2)
    plot((ref.time-state.time(1))/1e9,ref.y,'r-','LineWidth',1.2)
    legend('Reference', name+' result','FontSize',11,'interpreter', 'latex')
    %axis([0 (state.time(end)-state.time(1))/1e9 -0.01 0.25])
    
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('y (m)','FontSize',15,'interpreter', 'latex')
    grid on

    subplot(1,3,3)
    hold on
    plot((state.time-state.time(1))/1e9,state.theta,'b-','LineWidth',1.2)
    plot((ref.time-state.time(1))/1e9,ref.theta,'r-','LineWidth',1.2)
    legend('Reference', name+' result','FontSize',11,'interpreter', 'latex')
    %axis([0  -0.1 0.1])
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('theta (rad)','FontSize',15,'interpreter', 'latex')
    grid on
    
end

