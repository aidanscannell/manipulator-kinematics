function posEEfree = freeMotion(xyzp,N)
fprintf('\n\n------------------------------------- FREE MOTION TRAJECTORY -------------------------------------\n\n')
    pos = zeros(4,N*(size(xyzp,1)-1));
    kk = 1;
    posEEfree = [];
    for j = 1 : size(xyzp,1)-1
        % inverse kinematics to get joint configurations for task position i and i+1
        result1 = inverseKinematics(xyzp(j,1),xyzp(j,2),xyzp(j,3),xyzp(j,4));
        result2 = inverseKinematics(xyzp(j+1,1),xyzp(j+1,2),xyzp(j+1,3),xyzp(j+1,4));
        
        % create theta variables
        theta11 = getTheta(result1(1,1)); theta21 = getTheta(result1(1,2)); theta31 = getTheta(result1(1,3));
        theta41 = getTheta(result1(1,4)); theta51 = getTheta(result1(1,5)); theta12 = getTheta(result2(1,1));
        theta22 = getTheta(result2(1,2)); theta32 = getTheta(result2(1,3)); theta42 = getTheta(result2(1,4));
        theta52 = getTheta(result2(1,5));
        
        % calculate angle step size for each joint and pitch
%         pitch = linspace(xyzp(j,4),xyzp(j+1,4),N); % vector of pitch angles
        pitch = xyzp(j,4);
        pitch_d = (xyzp(j+1,4) - xyzp(j,4))/N; % vector of pitch angles 
        theta1_d = (theta12 - theta11)/N;
        theta2_d = (theta22 - theta21)/N;
        theta3_d = (theta32 - theta31)/N;
        theta4_d = (theta42 - theta41)/N;
        theta5_d = (theta52 - theta51)/N;
        
        for i = 1 : N                
            % function calculates compound transformation matrices
            [link1 link2 link3 link4 link5] = forwardKinematics(theta11,theta21,theta31,theta41,theta51);
            
            % plot arm position on the same figure
            clear p;
            p(:,:,i) = [[0,0,0,0];getP(link1),theta11;getP(link2),theta21;getP(link3),theta31;getP(link4),theta41;getP(link5),pitch];
            plotFK('Free Motion',link1,link2,link3,link4,link5,0,0)
            
            posEEfree = [posEEfree; getP(link5)];
            
            % print results
            fprintf('------------------------------------- Position %d -------------------------------------\n',i)
            fprintf('JOINT COORDINATES/ANGLES:\n')
            for j = 2:6
                fprintf('x%d = %.2f, y%d = %.2f, z%d = %.2f, theta%d = %.2f\n',j-1,p(j,1,i),j-1,p(j,2,i),j-1,p(j,3,i),j-1,p(j,4,i))
            end
            fprintf(' \n\n')
%             pause(0.2)
            
            % store joint angles for every time step
            pos(1,kk) = theta11;
            pos(2,kk) = theta21;
            pos(3,kk) = theta31;
            pos(4,kk) = theta41;
            pos(5,kk) = theta51;
            kk = kk + 1;

            % update angles for next iteration
            theta11 = theta11 + theta1_d;
            theta21 = theta21 + theta2_d;
            theta31 = theta31 + theta3_d;
            theta41 = theta41 + theta4_d;
            theta51 = theta51 + theta5_d;
            pitch = pitch + pitch_d;
                        
        end
    end
    
    % store end-effector positions and plot 
    posEEfree = [posEEfree;xyzp(end,1),xyzp(end,2),xyzp(end,3)];
    plot3(posEEfree(:,1),posEEfree(:,2),posEEfree(:,3),'*-')
    
    % Plot joint position
    plotPos(pos,kk)
    
    % Plot joint speed
    plotSpeed(pos,kk)
    
    % Plot joint acceleration
    plotAcc(pos,kk)
end

