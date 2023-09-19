function [ ] = plot_control(control,max_value)
    figure()
    subplot(3,2,1)
    hold on
    plot([0 (control.time(end)-control.time(1))/1e9],[0.028 0.028],'g-','LineWidth',1)
    plot([0 (control.time(end)-control.time(1))/1e9],[max_value max_value],'r-','LineWidth',1)
    plot((control.time-control.time(1))/1e9,control.motor_a_1,'b-','LineWidth',1)
    legend('Idle signal',"Max signal",'Motor $a_1$ signal','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('motor signal value ($\%$)','FontSize',15,'interpreter', 'latex')
    title("Motor $a_1$ signal",'FontSize',15,'interpreter', 'latex')
    axis([0 (control.time(end)-control.time(1))/1e9 0 max_value+0.07])
    grid on

    subplot(3,2,2)
    hold on
    plot([0 (control.time(end)-control.time(1))/1e9],[0.028 0.028],'g-','LineWidth',1)
    plot([0 (control.time(end)-control.time(1))/1e9],[max_value max_value],'r-','LineWidth',1)
    plot((control.time-control.time(1))/1e9,control.motor_a_2,'b-','LineWidth',1)
    legend('Idle signal',"Max signal",'Motor $a_2$ signal','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('motor signal value ($\%$)','FontSize',15,'interpreter', 'latex')
    title("Motor $a_2$ signal",'FontSize',15,'interpreter', 'latex')
    axis([0 (control.time(end)-control.time(1))/1e9 0 max_value+0.07])
    grid on
    

    subplot(3,2,3)
    hold on
    plot([0 (control.time(end)-control.time(1))/1e9],[0.028 0.028],'g-','LineWidth',1)
    plot([0 (control.time(end)-control.time(1))/1e9],[max_value max_value],'r-','LineWidth',1)
    plot((control.time-control.time(1))/1e9,control.motor_b_1,'b-','LineWidth',1)
    legend('Idle signal',"Max signal",'Motor $b_1$ signal','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('motor signal value ($\%$)','FontSize',15,'interpreter', 'latex')
    title("Motor $b_1$ signal",'FontSize',15,'interpreter', 'latex')
    axis([0 (control.time(end)-control.time(1))/1e9 0 max_value+0.07])
    grid on
    
    subplot(3,2,4)
    hold on
    plot([0 (control.time(end)-control.time(1))/1e9],[0.028 0.028],'g-','LineWidth',1)
    plot([0 (control.time(end)-control.time(1))/1e9],[max_value max_value],'r-','LineWidth',1)
    plot((control.time-control.time(1))/1e9,control.motor_b_2,'b-','LineWidth',1)
    legend('Idle signal',"Max signal",'Motor $b_2$ signal','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('motor signal value ($\%$)','FontSize',15,'interpreter', 'latex')
    title("Motor $b_2$ signal",'FontSize',15,'interpreter', 'latex')
    axis([0 (control.time(end)-control.time(1))/1e9 0 max_value+0.07])
    grid on
       
    subplot(3,2,5)
    hold on
    plot([0 (control.time(end)-control.time(1))/1e9],[0.028 0.028],'g-','LineWidth',1)
    plot([0 (control.time(end)-control.time(1))/1e9],[max_value max_value],'r-','LineWidth',1)
    plot((control.time-control.time(1))/1e9,control.motor_c_1,'b-','LineWidth',1)
    legend('Idle signal',"Max signal",'Motor $c_1$ signal','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('motor signal value ($\%$)','FontSize',15,'interpreter', 'latex')
    title("Motor $c_1$ signal",'FontSize',15,'interpreter', 'latex')
    axis([0 (control.time(end)-control.time(1))/1e9 0 max_value+0.07])
    grid on
    
    subplot(3,2,6)
    hold on
    plot([0 (control.time(end)-control.time(1))/1e9],[0.028 0.028],'g-','LineWidth',1)
    plot([0 (control.time(end)-control.time(1))/1e9],[max_value max_value],'r-','LineWidth',1)
    plot((control.time-control.time(1))/1e9,control.motor_c_2,'b-','LineWidth',1)
    legend('Idle signal',"Max signal",'Motor $c_2$ signal','FontSize',11,'interpreter', 'latex')
    xlabel('Time (s)','FontSize',15,'interpreter', 'latex')
    ylabel('motor signal value ($\%$)','FontSize',15,'interpreter', 'latex')
    title("Motor $c_2$ signal",'FontSize',15,'interpreter', 'latex')
    axis([0 (control.time(end)-control.time(1))/1e9 0 max_value+0.07])
    grid on
    
    %title("Position references and "+name+" results")
end

