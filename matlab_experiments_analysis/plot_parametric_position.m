function [ ] = plot_parametric_position(ref,state,name)
    figure()
    hold on
    plot(state.x,state.y,'b+')
    plot(ref.x,ref.y,'r-o','LineWidth',0.8)
    %quiver(ref.x,ref.y,cos(ref.theta),sin(ref.theta),0.1)
    axis([-0.25 0.25 -0.25 0.25])
    title("Reference and "+name+" results")
    legend(name+' result','Reference','FontSize',11,'interpreter', 'latex')
    xlabel('x (m)','FontSize',15,'interpreter', 'latex')
    ylabel('y (m)','FontSize',15,'interpreter', 'latex')
    grid on


end

