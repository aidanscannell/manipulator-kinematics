%  Planar parallel Kinematics. 
%  Robotic Fundamentals UFMF4X-15-M
%  paralellKinamtics.m
%
%  Created by AidanScannell.
%  Copyright © 2017 AidanScannell. All rights reserved.

%% Setup
clear all
clc
close all

arm; % import arm class

% set flags
IK_flag = 1;
workspace_flag = 1;

% Specify design parameters
s_a = 170; % lower section
L = 130; % upper section
r_p = 130; % platform joint circle radius
r_b = 290; % base joint circle radius

%% Inverse Kinematics 
if IK_flag == 1
    
    % Input parameters
    X_c = 200;
    Y_c = 150;
    a = -30;

    % Calculate global variables
    BC = [X_c-r_b*cosd(30);Y_c-r_b*sind(30);0]; % Define vector BC
    R_BC = [cosd(a), -sind(a), 0; sind(a), cosd(a), 0; 0, 0, 1]; % Define 2D rotation matrix for platform with rotation a

    % Create arm objects
    arm1 = arm(1,R_BC,BC,s_a,r_p,L,r_b,a,X_c,Y_c);
    arm2 = arm(2,R_BC,BC,s_a,r_p,L,r_b,a,X_c,Y_c);
    arm3 = arm(3,R_BC,BC,s_a,r_p,L,r_b,a,X_c,Y_c);
    
    % determine if target position is within workspace
    [x,y,ii,singularity_flag] = singularity(arm1,arm2,arm3,X_c,Y_c,1,[],[]);
    
    if singularity_flag == 0
        % Get joint coordinates
        posArm1 = getJointCo(arm1,s_a,L);
        posArm2 = getJointCo(arm2,s_a,L);
        posArm3 = getJointCo(arm3,s_a,L);

        % Create lists of arm parameters for looping
        arms = {getJointCo(arm1,s_a,L),getJointCo(arm2,s_a,L),getJointCo(arm3,s_a,L)};
        theta = {arm1.getTheta, arm2.getTheta, arm3.getTheta};
        psi = {arm1.getPsi, arm2.getPsi, arm3.getPsi};

        % Plot
        figure
        hold on; grid on  
%         title('Inverse Kinematics')    
        xlabel('x'); ylabel('y'); % axis labels
        axis equal;
        plot(X_c,Y_c,'X') % target position (for center of platform)
        tri_b = [0 2*r_b*cosd(30) r_b*cosd(30) 0; 0 0 (r_b*sind(30) + r_b) 0]; % base triangle
        tri_p = [posArm1(1,3) posArm2(1,3) posArm3(1,3) posArm1(1,3); posArm1(2,3) posArm2(2,3) posArm3(2,3) posArm1(2,3)]; % platform triangle
        plot(tri_p(1,:), tri_p(2,:)) % plot base
        plot(tri_b(1,:), tri_b(2,:)) % plot platform

        for i = 1:3    
            text(arms{i}(1,3),arms{i}(2,3),num2str(i)); % add PP label
            text(arms{i}(1,1),arms{i}(2,1),num2str(theta{i}(1))); % add joint theta values
            text(arms{i}(1,2),arms{i}(2,2),num2str(psi{i}(1))); % add joint psi values
            plot(arms{i}(1,:),arms{i}(2,:)); % plot arm
        end
    else
        fprintf('Target position outside of workspace (singularity)\n')
    end
end

%% Workspace 
if workspace_flag == 1
    
    % Input parameters
    x = zeros(1,120);
    y = zeros(1,120);
    a = 0;

    % Calculate workspace by neglecting singularities
    ii = 1;
    for X_c = 5:5:600
        for Y_c = 5:5:600
            % Calculate global variables
            BC = [X_c-r_b*cosd(30);Y_c-r_b*sind(30);0]; % Define vector BC
            R_BC = [cosd(a), -sind(a), 0; sind(a), cosd(a), 0; 0, 0, 1]; % Define 2D rotation matrix for platform with rotation a

            % Create arm objects
            arm1 = arm(1,R_BC,BC,s_a,r_p,L,r_b,a,X_c,Y_c);
            arm2 = arm(2,R_BC,BC,s_a,r_p,L,r_b,a,X_c,Y_c);
            arm3 = arm(3,R_BC,BC,s_a,r_p,L,r_b,a,X_c,Y_c);
            
            % Determine which points lie within the workspace
            [x,y,ii,singularity_flag] = singularity(arm1,arm2,arm3,X_c,Y_c,ii,x,y);
            
        end
    end
        
    % Determine workspace by plotting range of each joint
    jj = 1;
    for theta = 1:5:360
        for psi = 1:5:360
            
            % x,y positions for each joint
            x1(jj) = s_a*cosd(theta) + L*cosd(psi) + r_p*cosd(30-a);
            x2(jj) = s_a*cosd(theta+120) + L*cosd(psi+120) + r_p*cosd(30+120-a) + 2*r_b*cosd(30);
            x3(jj) = s_a*cosd(theta+240) + L*cosd(psi+240) + r_p*cosd(30+240-a) + r_b*cosd(30);                         
            y1(jj) = s_a*sind(theta) + L*sind(psi) + r_p*sind(30-a);
            y2(jj) = s_a*sind(theta+120) + L*sind(psi+120) + r_p*sind(30+120-a);
            y3(jj) = s_a*sind(theta+240) + L*sind(psi+240) + r_p*sind(30+240-a) + (r_b*sind(30) + r_b);      
            
            jj = jj + 1;
        end
    end

    % Plot
    figure
    hold off; hold on; grid on    
%     title(['Workspace (a = ' num2str(a) ')'])    
    xlabel('x'); ylabel('y'); % axis labels
    axis([-10 550 -10 450])
    tri_b = [0 2*r_b*cosd(30) r_b*cosd(30) 0; 0 0 (r_b*sind(30) + r_b) 0]; % base triangle
    plot(tri_b(1,:), tri_b(2,:)) % plot base
    plot(x,y,'o') % target position (for center of platform)
    
    % Plot
    figure
    hold off; hold on; grid on    
%     title(['Workspace (a = ' num2str(a) ')'])    
    xlabel('x'); ylabel('y'); % axis labels
    axis equal
    tri_b = [0 2*r_b*cosd(30) r_b*cosd(30) 0; 0 0 (r_b*sind(30) + r_b) 0]; % base triangle
    plot(tri_b(1,:), tri_b(2,:)) % plot base
    plot(x1,y1,'.r','MarkerSize',1) % target position (for center of platform)
    plot(x2,y2,'.g','MarkerSize',1) % target position (for center of platform)
    plot(x3,y3,'.b','MarkerSize',1) % target position (for center of platform)

end

%% Singularity function
function [x,y,ii,singularity_flag] = singularity(arm1,arm2,arm3,X_c,Y_c,ii,x,y)
%	singularity -> determines if the target position is within the
%	workspace for the given arm links
%   
%   by Aidan Scannell
%
%	===== Inputs =====
%	armN - armLink object N
%   X_c - target x position
%   Y_c - target y position
%   ii - iteration number for workspace plotting
%   x - vector containing x positions in workspace
%   y - vector containing y positions in workspace
%
%	===== Outputs ====
%   ii - iteration number for workspace plotting
%   x - vector containing x positions in workspace
%   y - vector containing y positions in workspace
%   singularity_flag - 0 if not singularity, 1 if singularity
    singularity_flag = 1;
    theta1 = getTheta(arm1);
    theta2 = getTheta(arm2);
    theta3 = getTheta(arm3);

    psi1 = getPsi(arm1);
    psi2 = getPsi(arm2);
    psi3 = getPsi(arm3);
    if (isnan(theta1(1)) == 0) && (isnan(theta2(1)) == 0) && (isnan(theta3(1)) == 0) && ...
            (isnan(psi1) == 0) && (isnan(psi2) == 0) && (isnan(psi3) == 0)
        x(ii) = X_c;
        y(ii) = Y_c;
        ii = ii + 1;
        singularity_flag = 0;
    end 
    if (isnan(theta1(2)) == 0) && (isnan(theta2(2)) == 0) && (isnan(theta3(2)) == 0) && ...
            (isnan(psi1) == 0) && (isnan(psi2) == 0) && (isnan(psi3) == 0)
        x(ii) = X_c;
        y(ii) = Y_c;
        ii = ii + 1; 
        singularity_flag = 0;
    end 
end