function posEEobs = obsAvoid(xyzp,N,time_steps) 
    fprintf('\n\n------------------------------------- OBSTACLE AVOIDANCE TRAJECTORY -------------------------------------\n\n')
    hold off; figure;
    pos = zeros(4,N*(size(xyzp,1)-1));
    kk = 1;
    posEEobs = [];
%     N = 10;
%     time_steps = 20;
%     xyzp = [0;0;0;0];
%     for j = 1 : size(position,1)-1
%         
%         % create vectors of x,y,z,p positions for N steps
%         x = linspace(position(j,1),position(j+1,1),N); % vector of x positions
%         y = linspace(position(j,2),position(j+1,2),N); % vector of y positions
%         z = linspace(position(j,3),position(j+1,3),N); % vector of z positions
%         pitch = linspace(position(j,4),position(j+1,4),N); % vector of pitch angles  
%     
%         xyzp = [xyzp(1,:), x(2:end); xyzp(2,:), y(2:end); xyzp(3,:), z(2:end); xyzp(4,:), pitch(2:end)];
%     end
%     xyzp = xyzp(:,2:end);
%     for j = 1 : size(xyzp,2)
%         % inverse kinematics to get joint configurations for task position i and i+1
%         result = inverseKinematics(xyzp(1,j),xyzp(2,j),xyzp(3,j),xyzp(4,j));
%         
%         % create theta variables
%         for i = 1:5
%             theta(i,j) = getTheta(result(1,i));
%         end
%     end
%     for j = 1 : size(xyzp,2)-1
%         for i = 1:5            
%             q_0 = theta(i,j);
%             q_f = theta(i,j+1);
%             t_0 = 0;
%             t_f = 1;
%             v_0 = 0;
%             v_f = 0;
%             a_0 = 0;
%             a_f = 0;
% 
%             A = [1,  t_0,  t_0^2,  t_0^3,   t_0^4,    t_0^5;
%                  0,  1,    2*t_0,  3*t_0^3, 4*t_0^3,  5*t_0^4;
%                  0,  0,    2,      6*t_0,   12*t_0^2, 20*t_0^3; 
%                  1,  t_f,  t_f^2,  t_f^3,   t_f^4,    t_f^5;
%                  0,  1,    2*t_f,  3*t_f^2, 4*t_f^3,  5*t_f^4;
%                  0,  0,    0,      6*t_f,   12*t_f^2, 20*t_f^3];
% 
%             B = [q_0; v_0; a_0; q_f; v_f; a_f]; 
% 
%             X{j}(i,:) = linsolve(A,B);
%         end
%     end
%     
%     % iterate through time steps    
%     x = linspace(0,t_f,time_steps);
%     for j = 1 : size(xyzp,1)-1
%         for ii = 1 : time_steps           
% 
%             for i = 1:5
%                 theta_(i) = X{j}(i,6)*x(ii)^5 + X{j}(i,5)*x(ii)^4 + X{j}(i,4)*x(ii)^3 + ...
%                     X{j}(i,3)*x(ii)^2 + X{j}(i,2)*x(ii)^1 + X{j}(i,1);
%             end
%             
%             % function calculates compound transformation matrices
%             [link1 link2 link3 link4 link5] = forwardKinematics(theta_(1),theta_(2),theta_(3),theta_(4),theta_(5));
% 
%             % plot arm position on the same figure
%             plotFK('Quintic Motion',link1,link2,link3,link4,link5,0,0)
% 
%             % store joint angles for every time step
%             pos(1,kk) = theta_(1);
%             pos(2,kk) = theta_(2);
%             pos(3,kk) = theta_(3);
%             pos(4,kk) = theta_(4);
%             pos(5,kk) = theta_(5);
%             kk = kk + 1;
% 
%             % store end-effector position 
%             posEEobs = [posEEobs; getP(link5)];
%         end
%     end
%     
%     % plot straight line between points
%     hold on
%     plot3(posEEobs(:,1),posEEobs(:,2),posEEobs(:,3),'*-')
%     
%     % plot cylinder
%     [X,Y,Z] = cylinder(40,100);
%     plot3(X+150,Y+150,Z*400,'s-')
% 
%     % Plot joint position
%     plotPos(pos,kk)
%     
%     % Plot joint speed
%     plotSpeed(pos,kk)
%     
%     % Plot joint acc
%     plotAcc(pos,kk)
    
    for j = 1 : size(xyzp,1)-1
        
        % create vectors of x,y,z,p positions for N steps
        x = linspace(xyzp(j,1),xyzp(j+1,1),N); % vector of x positions
        y = linspace(xyzp(j,2),xyzp(j+1,2),N); % vector of y positions
        z = linspace(xyzp(j,3),xyzp(j+1,3),N); % vector of z positions
        pitch = linspace(xyzp(j,4),xyzp(j+1,4),N); % vector of pitch angles  
    
        position = [x;y;z;pitch]';
        clear p;
        for i = 1:length(position)-1
            fprintf('\n------------------------------------- Position %d -------------------------------------\n',i)

            % perform inverse kinematics            
            result = inverseKinematics(position(i,1),position(i,2),position(i,3),position(i,4))

            % plot arm position on the same figure
            p(:,:,i) = [[0,0,0,0];getP(result(1,1)),getTheta(result(1,1));getP(result(1,2)),...
                getTheta(result(1,2));getP(result(1,3)),getTheta(result(1,3));getP(result(1,4)),...
                getTheta(result(1,4));getP(result(1,5)),getTheta(result(1,5))];
            plotFK('Obstacle Avoidance Trajectory',result(1,1),result(1,2),result(1,3),result(1,4),result(1,5),0,0)

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
            posEEobs = [posEEobs; getP(link5)];
        end



    end

    % plot straight line between points
    hold on
    plot3(posEEobs(:,1),posEEobs(:,2),posEEobs(:,3),'*-')
    
    [X,Y,Z] = cylinder(40,100);
    plot3(X+150,Y+150,Z*400,'s-')
    
    % Plot joint position
    plotPos(pos,kk)
    
    % Plot joint speed
    plotSpeed(pos,kk)
    
    % Plot joint acceleration
    plotAcc(pos,kk)
end

