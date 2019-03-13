function workspace(range1,range2,range3,range4,N_steps)
%	workspace -> Plots workspace for lynxmotion arm
%   
%   by Aidan Scannell
%
%	===== Inputs =====
%	rangeN - range of joint N
%	N_steps - number of steps within each joints range

%	===== Outputs ====
%	prints number of iterations
%   plots 3D workspace

    %% calcualte range step sizes for given N
    step1 = diff(range1)/(N_steps-1);
    step2 = diff(range2)/(N_steps-1);
    step3 = diff(range3)/(N_steps-1);
    step4 = diff(range4)/(N_steps-1);
    
    %% Loop through joint angles and store end-effector coordinates
    
    ii = 1; % counter
    
    % initialise x, y, z arrays to speed up computation
    N = N_steps^4 + 1;
    x = zeros(N,1); y = zeros(N,1); z = zeros(N,1);
    
    % matrix to plot different rgb intensities for each point
    fill = zeros(N,3);
    
    i = 1;
    for q1 = range1(1):step1:range1(2)
        fprintf('Workspace Iteration = %d\n', i)
        for q2 = range2(1):step2:range2(2)
            for q3 = range3(1):step3:range3(2)
                for q4 = range4(1):step4:range4(2)
                    [link1 link2 link3 link4 link5] = forwardKinematics(q1,q2,q3,q4,0); % initialise objects

                    p = [getP(link5)];
%                     range1 = [-90,90]; range2 = [0,180]; range3 = [-160,0]; range4 = [-130,90]; range5 = [0,360]; % set joint ranges
% range4 = [-90,90];   
% range1 = [-90,90]; range2 = [0,180]; range3 = [-165,0]; range4 = [-180,0]; range5 = [0,360]; % set joint ranges
                    fill(ii,1) = 1 - (q2 / diff(range2));
                    fill(ii,2) = ((q3+diff(range3))/diff(range3));
                    fill(ii,3) = ((q4+range4(2))/diff(range4));
                    x(ii) = p(1);
                    y(ii) = p(2);
                    z(ii) = p(3);

                    ii = ii + 1; % increment counter
                end
            end
        end
        i = i + 1;
    end

    % print number of iterations
    fprintf('Number of iterations = %d\n',ii);

    %% Plot workspace
    hold off; hold on
    figure
    title('Workspace')    
    scatter3(x,y,z,10,fill,'filled')
    xlabel('x'); ylabel('y'); zlabel('z') % axis labels
%     hold off; hold on
%     figure
%     scatter(x,y,10,fill,'filled')
%     hold off; hold on
%     figure
%     scatter(y,z,10,fill,'filled')
%     hold off; hold on
%     figure
%     scatter(x,z,10,fill,'filled')

%     hold off; hold on;
%     iy = y(x >= 0);
%     iz = z(x >= 0);
%     figure
%     scatter(iy, iz)
%     grid on
%     title('Plot Z(Y,X=1)')
%     ix = x(y >= 0);
%     iz = z(y >= 0);
%     hold off; hold on;
%     figure
%     scatter(ix, iz)
%     grid on
%     title('Plot Z(X,Y=3*\pi/4)')
%     hold off   

end

