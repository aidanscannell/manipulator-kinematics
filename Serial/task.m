function [pos, kk] = task(xyzp,title,pos,kk)
%	task -> performs inverse kinemtics for each position/orientation in
%	matrix xyzp and prints results
%   
%   by Aidan Scannell
%
%	===== Inputs =====
%	xyzp - matrix containing array of target positions/orientations [x; y; z; pitch]
%   title - string containing title of figure
%
%	===== Outputs ====
%	prints target position and joint coordinates and angles
    for i = 1:length(xyzp)
        fprintf('\n------------------------------------- Position %d -------------------------------------\n',i)
        
        % perform inverse kinematics
        result = inverseKinematics(xyzp(i,1),xyzp(i,2),xyzp(i,3),xyzp(i,4));
        
        % plot arm position on the same figure
        p(:,:,i) = [[0,0,0,0];getP(result(1,1)),getTheta(result(1,1));getP(result(1,2)),...
            getTheta(result(1,2));getP(result(1,3)),getTheta(result(1,3));getP(result(1,4)),...
            getTheta(result(1,4));getP(result(1,5)),getTheta(result(1,5))];
        plotFK(title,result(1,1),result(1,2),result(1,3),result(1,4),result(1,5),0,0)

        % print results        
        fprintf('TARGET: \nx = %d, y = %d, z = %d, pitch = %d\n\n',xyzp(i,1),xyzp(i,2),xyzp(i,3),xyzp(i,4))
        fprintf('JOINT COORDINATES/ANGLES:\n')
        for j = 2:6
            fprintf('x%d = %.2f, y%d = %.2f, z%d = %.2f, theta%d = %.2f\n',j-1,p(j,1,i),j-1,p(j,2,i),j-1,p(j,3,i),j-1,p(j,4,i))
        end
        fprintf(' \n\n')
%         pause(0.2)
        
        % store joint angles for every time step
        pos(1,kk) = getTheta(result(1,1));
        pos(2,kk) = getTheta(result(1,2));
        pos(3,kk) = getTheta(result(1,3));
        pos(4,kk) = getTheta(result(1,4));
        pos(5,kk) = getTheta(result(1,5));
        kk = kk + 1;
    end
end