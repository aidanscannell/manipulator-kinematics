function plotFK(t,link1,link2,link3,link4,link5,new_figure_flag,joint_angles_flag)
%	plotFK -> Plot the robotic arm in 3D space for given foward kinematics
%   
%   by Aidan Scannell
%
%	===== Inputs =====
%	linkN - armLink object for link N

%	===== Outputs ====
%	plots arm in 3D space
    if new_figure_flag == 1
        figure
    end
    title(t)    
    hold on; grid on
    xlabel('x'); ylabel('y'); zlabel('z') % z-axis label
    theta1 = getTheta(link1); theta2 = getTheta(link2); theta3 = getTheta(link3); theta4 = getTheta(link4);
    theta5 = getTheta(link5);
    p1 = getP(link1); p2 = getP(link2); p3 = getP(link3); p4 = getP(link4); p5 = getP(link5);
    p = [[0,0,0];getP(link1);getP(link2);getP(link3);getP(link4);getP(link5)];
    if joint_angles_flag == 1
        text(0,0,0,num2str(theta1,4));
        text(p2(1),p2(2),p2(3),num2str(theta2,4));
        text(p3(1),p3(2),p3(3),num2str(theta3,4));
        text(p4(1),p4(2),p4(3),num2str(theta4,4));
        text(p5(1),p5(2),p5(3),num2str(theta5,4));
    end

%     text(p5(1),p5(2),p5(3),['pos ',num2str(iii)]);
%     grid on
%     daspect([1 1 1])
%     minx = min([0, p2(1), p3(1), p4(1), p5(1)]);
%     maxx = max([0, p2(1), p3(1), p4(1), p5(1)]);
%     miny = min([0, p2(2), p3(2), p4(2), p5(2)]);
%     maxy = max([0, p2(2), p3(2), p4(2), p5(2)]);
%     minz = min([0, p2(3), p3(3), p4(3), p5(3)]);
%     maxz = max([0, p2(3), p3(3), p4(3), p5(3)]);
%     minA = min([minx, miny, minz]);
%     maxA = max([maxx, maxy, maxz]);
%     axis([minA maxA minA maxA minA maxA])
    plot3(p(:,1),p(:,2),p(:,3), 'b')
end
