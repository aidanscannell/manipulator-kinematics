function parBlendTraj(xyzp,N)
    fprintf('\n\n------------------------------------- LINEAR WITH PARABOLIC BLEND TRAJECTORY -------------------------------------\n\n')
    hold off; figure; hold on;
    pos = zeros(4,N*(size(xyzp,1)-1));
    kk = 1;
%     positions = [0;0;0;0];
%     for j = 1 : size(xyzp,1)-1
%         
%         % create vectors of x,y,z,p positions for N steps
%         x = linspace(xyzp(j,1),xyzp(j+1,1),N); % vector of x positions
%         y = linspace(xyzp(j,2),xyzp(j+1,2),N); % vector of y positions
%         z = linspace(xyzp(j,3),xyzp(j+1,3),N); % vector of z positions
%         p = linspace(xyzp(j,4),xyzp(j+1,4),N); % vector of pitch angles  
%         
%         positions = [positions(1,:), x(2:end); positions(2,:), y(2:end); positions(3,:), z(2:end); positions(4,:), p(2:end)];                         
% 
%     end
%     positions = positions(:,2:end);
%     
%     for j = 1 : size(positions,2)
%         % inverse kinematics to get joint configurations for task position i and i+1
%         result{j} = inverseKinematics(positions(1,j),positions(2,j),positions(3,j),positions(4,j));      
%     end
    
    for j = 1 : size(xyzp,1)
        % inverse kinematics to get joint configurations for task position i and i+1
        result{j} = inverseKinematics(xyzp(j,1),xyzp(j,2),xyzp(j,3),xyzp(j,4));      
    end
    % create theta variables
    for i = 1:5
        for j = 1:size(xyzp,1)
            theta(i,j) = getTheta(result{j}(1,i));
%             if i == 4 
%                 theta(i,j) = getTheta(result{j}(1,i)) + 90; 
%             end 
        end        
    end 
    
    a = 20;
    v = 5;
    for j = 1:size(xyzp,1)-1
        for i = 1:5 
            if j == 1
                t_d(i,j) = abs(theta(i,j) / v);
%                 a_sign(i,j) = sign(theta(i,j));
            else
                t_d(i,j) = abs((theta(i,j) - theta(i,j-1)) / v);
%                 a_sign(i,j) = sign(theta(i,j) - theta(i,j-1));
            end            
        end
    end    
    
    % calculate t_d times
    for j = 1:size(xyzp,1)-1
        t_f(j) = max(t_d(:,j));
    end
    
    for j = 1:size(xyzp,1)-1
        for i = 1:5 
            if j == 1
                theta_d(i,j) = theta(i,j) / t_f(j);
            else
                theta_d(i,j) = (theta(i,j) - theta(i,j-1)) / t_f(j);
            end            
        end
    end 
    
        
    % calculate blend times
    for j = 1:size(xyzp,1)-1
        for i = 1:5             
            if j == 1
                t_b(i,j) = (0 - theta(i,j) + v * t_f(j)) / v;
            else
                t_b(i,j) = (theta(i,j-1) - theta(i,j) + v * t_f(j)) / v;
            end              
        end
    end
         
    % loop through target positions
    for j = 1 : size(xyzp,1)-2
        
        for time = 1 : 0.1 : t_f(j)
            for i = 1:4
                if time < t_b(i,j)
                    q(i) = theta(i,j) + a/2 * time^2;
                    fprintf('blend 1\n')
                end
                if time >= t_b(i,j)
                    if j==1 
                        q(i) = (theta(i,j) - v * t_f(j)) / 2 + v*time;
                    else
                        q(i) = (theta(i,j-1) + theta(i,j) - v * t_f(j)) / 2 + v*time;
                    end
                    fprintf('lin\n')
                end
                if time > (t_f(j)-t_b(i,j))
                    q(i) = theta(i,j) - (a*t_f(j)^2)/2 + a * t_f(j)*time - a/2*time^2;  
                    fprintf('blend 2\n')
                end
                fprintf('t_b = %.2f\n',t_b(i,j))
                fprintf('t_f = %.2f\n',t_f(j))

%                 theta_(i) = getTheta(result{j}(1,i)) + theta_inc(i);
                theta_(i) = q(i)
                

            end
            
            % function calculates compound transformation matrices
            [link1 link2 link3 link4 link5] = forwardKinematics(theta_(1),theta_(2),theta_(3),theta_(4),0);
            
            % plot arm position on the same figure
            plotFK('Linear Parabolic Blend Motion',link1,link2,link3,link4,link5,0,0)            
            
            % store joint angles for every time step
            pos(1,kk) = theta_(1);
            pos(2,kk) = theta_(2);
            pos(3,kk) = theta_(3);
            pos(4,kk) = theta_(4);
            pos(5,kk) = theta_(4);
            kk = kk + 1;
        end
    end
    
    
      
    % Plot joint position
    plotPos(theta,size(theta,2)+1)
    
    % Plot joint speed
    plotSpeed(theta,size(theta,2)+1)
end

