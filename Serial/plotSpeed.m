function plotSpeed(pos,kk)
    % Plot joint position
    dy1 = diff(pos(1,:));% ./ diff([0:kk-2]);
    dy2 = diff(pos(2,:));% ./ diff([0:kk-2]);
    dy3 = diff(pos(3,:));% ./ diff([0:kk-2]);
    dy4 = diff(pos(4,:));% ./ diff([0:kk-2]);
    dy5 = diff(pos(5,:));% ./ diff([0:kk-2]);
    
    hold off
    figure
    hold on
    plot([0:size(pos(1,:),2)-2],dy1,'->')
    plot([0:size(pos(1,:),2)-2],dy2,'-*')
    plot([0:size(pos(1,:),2)-2],dy3,'-s')
    plot([0:size(pos(1,:),2)-2],dy4,'-d')
%     plot([2:kk-1],dy5,'-^')
    xlabel('Time Step, i')
    ylabel('Joint Speed, (degrees/s)')
    legend('Joint 1','Joint 2','Joint 3','Joint 4')
%     legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5')
end
