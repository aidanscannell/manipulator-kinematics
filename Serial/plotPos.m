function plotPos(pos,kk)
    % Plot joint position
    hold off
    figure
    hold on
    plot([0:size(pos(1,:),2)-1],pos(1,:),'->')
    plot([0:size(pos(1,:),2)-1],pos(2,:),'-*')
    plot([0:size(pos(1,:),2)-1],pos(3,:),'-s')
    plot([0:size(pos(1,:),2)-1],pos(4,:),'-d')
%     plot([1:kk-1],pos(5,:),'-^')
    xlabel('Time Step, i')
    ylabel('Joint Position, (degrees)')
    legend('Joint 1','Joint 2','Joint 3','Joint 4')
%     legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5')
end
