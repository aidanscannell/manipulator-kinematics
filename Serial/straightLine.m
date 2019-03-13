function posEElin = straightLine(xyzp,N)
    fprintf('\n\n------------------------------------- STRAIGHT LINE TRAJECTORY -------------------------------------\n\n')
    hold off; figure;
    pos = zeros(4,N*(size(xyzp,1)-1));
    kk = 1;
    posEElin = [];
    
    for j = 1 : size(xyzp,1)-1
        
        % create vectors of x,y,z,p positions for N steps
        x = linspace(xyzp(j,1),xyzp(j+1,1),N); % vector of x positions
        y = linspace(xyzp(j,2),xyzp(j+1,2),N); % vector of y positions
        z = linspace(xyzp(j,3),xyzp(j+1,3),N); % vector of z positions
        p = linspace(xyzp(j,4),xyzp(j+1,4),N); % vector of pitch angles  
    
        position = [x;y;z;p]';
        clear p;
        for i = 1:length(position)
            fprintf('\n------------------------------------- Position %d -------------------------------------\n',i)

            % perform inverse kinematics
            result = inverseKinematics(position(i,1),position(i,2),position(i,3),position(i,4))

            % plot arm position on the same figure
            p(:,:,i) = [[0,0,0,0];getP(result(1,1)),getTheta(result(1,1));getP(result(1,2)),...
                getTheta(result(1,2));getP(result(1,3)),getTheta(result(1,3));getP(result(1,4)),...
                getTheta(result(1,4));getP(result(1,5)),getTheta(result(1,5))];
            plotFK('Straight Line Trajectory',result(1,1),result(1,2),result(1,3),result(1,4),result(1,5),0,0)

            % print results        
            fprintf('TARGET: \nx = %d, y = %d, z = %d, pitch = %d\n\n',position(i,1),position(i,2),...
                position(i,3),position(i,4))
            fprintf('JOINT COORDINATES/ANGLES:\n')
            for j = 2:6
                fprintf('x%d = %.2f, y%d = %.2f, z%d = %.2f, theta%d = %.2f\n',j-1,p(j,1,i),j-1,...
                    p(j,2,i),j-1,p(j,3,i),j-1,p(j,4,i))
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
            
            % store positions of end effector
            [link1 link2 link3 link4 link5] = forwardKinematics(getTheta(result(1,1)),getTheta(result(1,2)),...
                getTheta(result(1,3)),getTheta(result(1,4)),getTheta(result(1,5)));
            posEElin = [posEElin; getP(link5)];
        end



    end

    % plot straight line between points
    hold on
    plot3(posEElin(:,1),posEElin(:,2),posEElin(:,3),'*-')
    
    % Plot joint position
    plotPos(pos,kk)
    
    % Plot joint speed
    plotSpeed(pos,kk)
    
    % Plot joint acceleration
    plotAcc(pos,kk)
end
