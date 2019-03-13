function posEEpoly = polynomialTraj(xyzp,time_steps,N)
    fprintf('\n\n------------------------------------- POLYNOMIAL TRAJECTORY -------------------------------------\n\n')
    hold off; figure;
    pos = zeros(4,N*(size(xyzp,1)-1));
    kk = 1;
    posEEpoly = [];
    for j = 1 : size(xyzp,1)
        % inverse kinematics to get joint configurations for task position i and i+1
        result = inverseKinematics(xyzp(j,1),xyzp(j,2),xyzp(j,3),xyzp(j,4));
        
        % create theta variables
        for i = 1:5
            theta(i,j) = getTheta(result(1,i));
        end
    
    end
    
    for j = 1 : size(xyzp,1)-1
        for i = 1:5            
            q_0 = theta(i,j);
            q_f = theta(i,j+1);
            t_0 = 0;
            t_f = 1;
            v_0 = 0;
            v_f = 0;

            A = [1,  t_0,  t_0^2,  t_0^3;
                 0,  1,    2*t_0,  3*t_0^3;
                 1,  t_f,  t_f^2,  t_f^3;
                 0,  1,    2*t_f,  3*t_f^2;];

            B = [q_0; v_0; q_f; v_f]; 

            X{j}(i,:) = linsolve(A,B);
        end
    end
    
    % iterate through time steps    
    x = linspace(0,t_f,time_steps);
    for j = 1 : size(xyzp,1)-1
        for ii = 1 : time_steps           

            for i = 1:5
                theta_(i) = X{j}(i,4)*x(ii)^3 + X{j}(i,3)*x(ii)^2 + X{j}(i,2)*x(ii)^1 + X{j}(i,1);
            end
            
            % function calculates compound transformation matrices
            [link1 link2 link3 link4 link5] = forwardKinematics(theta_(1),theta_(2),theta_(3),theta_(4),theta_(5));

            % plot arm position on the same figure
            plotFK('Cubic Polynomial Motion',link1,link2,link3,link4,link5,0,0)

            % store joint angles for every time step
            pos(1,kk) = theta_(1);
            pos(2,kk) = theta_(2);
            pos(3,kk) = theta_(3);
            pos(4,kk) = theta_(4);
            pos(5,kk) = theta_(5);
            kk = kk + 1;

            % store end-effector position 
            posEEpoly = [posEEpoly; getP(link5)];
        end
    end
    
    % plot straight line between points
    hold on
    plot3(posEEpoly(:,1),posEEpoly(:,2),posEEpoly(:,3),'*-')
    
    % Plot joint position
    plotPos(pos,kk)
    
    % Plot joint speed
    plotSpeed(pos,kk)
    
    % Plot joint acc
    plotAcc(pos,kk)
end

