function plotAcc(pos,kk)
    % Plot joint position
    dy1 = diff(pos(1,:));% ./ diff([1:kk-1]);
    dy2 = diff(pos(2,:));% ./ diff([1:kk-1]);
    dy3 = diff(pos(3,:));% ./ diff([1:kk-1]);
    dy4 = diff(pos(4,:));% ./ diff([1:kk-1]);
    dy5 = diff(pos(5,:));% ./ diff([1:kk-1]);
    
%     dy12 = diff(dy1)
%     dy22 = diff(dy2)
%     dy32 = diff(dy3)
%     dy42 = diff(dy4)
%     dy52 = diff(dy5)
    
    dy12 = diff(dy1);% ./ diff([1:kk-2]);
    dy22 = diff(dy2);% ./ diff([1:kk-2]);
    dy32 = diff(dy3);% ./ diff([1:kk-2]);
    dy42 = diff(dy4);% ./ diff([1:kk-2]);
    dy52 = diff(dy5);% ./ diff([1:kk-2]);
    
    hold off
    figure
    hold on
    plot([0:size(pos(1,:),2)-3],dy12,'->')
    plot([0:size(pos(1,:),2)-3],dy22,'-*')
    plot([0:size(pos(1,:),2)-3],dy32,'-s')
    plot([0:size(pos(1,:),2)-3],dy42,'-d')
%     plot([2:kk-1],dy5,'-^')
    xlabel('Time Step, i')
    ylabel('Joint Acceleration, (degrees/s^2)')
    legend('Joint 1','Joint 2','Joint 3','Joint 4')
%     legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5')
end