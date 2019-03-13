%  Serial Kinematics for the lnyxmotion arm. 
%  Robotic Fundamentals UFMF4X-15-M
%  serialKinamtics.m
%
%  Instructions for running this code: 
%               1. set flags in input section ( 1 = run, 0 = don't run)
%               2. set variables in input section
%               3. if forward kinematics is run then the resulting end-effector position 
%               will be fed as an input to the inverse kinematics and override the values 
%               in the vector xyzpIK.
%
%  Created by AidanScannell.
%  Copyright © 2017 AidanScannell. All rights reserved.
clear all
clc
close all

%% Input 
% set flags
FK_flag = 1;
workspace_flag = 1;
IK_flag = 1;
task_flag = 1;
free_motion_flag = 1;
straight_line_motion_flag = 1;
poly_motion_flag = 1;
quintic_poly_motion_flag = 1;

lpbb_motion_flag = 0;
obstacle_avoidance_flag = 1;

% set end-effector position/orientation for inverse kinematics
xyzpIK = [100,0,200,-90]; % [x, y, z, pitch]

% set number of steps within each joints range
N_steps = 10; 

% set number of steps between task points for polynomial trajectories
time_steps = 30; 

% setup variables for task 
xyzp = [0,300,0,-90;0,250,300,0;250,0,300,0;0,-250,300,0;0,-300,0,-90]; % positions/orientations
N = 10; % number of steps between each position for trajectories

%% Setup
armLink; % import arm link class

global l1 l2 l3 l4 
global range1 range2 range3 range4 range5

% set up lynxmotion arm parameters
l1 = 63; l2 = 153; l3 = 153; l4 = 98; % set arm lengths
range1 = [-90,90]; range2 = [0,135]; range3 = [-145,0]; range4 = [-90,90]; range5 = [0,360]; % set joint ranges  

% initialise arrays for trajectory end-effector positions
posEEfree = [];
posEElin = [];
posEEpoly = [];
posEEquint = [];

%% Forward Kinematics 
if FK_flag == 1 
    % Initialise arm link agnles (select random joint angles within joint ranges)
    q1 =  range1(1) + (range1(2)-range1(1)).*rand(1);
    q2 =  range2(1) + (range2(2)-range2(1)).*rand(1);
    q3 =  range3(1) + (range3(2)-range3(1)).*rand(1);
    q4 =  range4(1) + (range4(2)-range4(1)).*rand(1);
    q5 =  range5(1) + (range5(2)-range5(1)).*rand(1);
%     q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = 0;
%     q1 = -25.27; q2 = 100.5; q3 = -41.19; q4 = -13.62; q5 = 154.6;
    
    % function calculates compound transformation matrices
    [link1 link2 link3 link4 link5] = forwardKinematics(q1,q2,q3,q4,q5);

    % print FK input and output
    fprintf('FK input angles: theta 1 = %.2f, theta 2 = %.2f, theta 3 = %.2f, theta 4 = %.2f, theta 5 = %.2f\n',q1,q2,q3,q4,q5)
    FK_p1 = getP(link1); FK_p2 = getP(link2); FK_p3 = getP(link3); FK_p4 = getP(link4); FK_p5 = getP(link5);
    L_pitch = sqrt( (FK_p5(2) - FK_p4(2))^2 + (FK_p5(1) - FK_p4(1))^2 ); % calculate length of link 4-5 in XY plane
    FK_pitch = atan2d( (FK_p5(3) - FK_p4(3)), L_pitch ); % calculate pitch
    fprintf('FK end-effector position: x = %.2f, y = %.2f, z = %.2f, pitch = %.2f\n',FK_p5(1),FK_p5(2),FK_p5(3), FK_pitch)  
    
    % Plot the robotic arm in 3D space
    plotFK('Forward Kinematics',link1,link2,link3,link4,link5,1,1)

end

%% Workspace
if workspace_flag == 1    
    workspace(range1,range2,range3,range4,N_steps);
end

%% Inverse Kinematics
if IK_flag == 1

    % set input of inverse kinematics to result of forward kinematcs otherwise to default 
    if exist('FK_p5','var')
        x = FK_p5(1); y = FK_p5(2); z = FK_p5(3); pitch = FK_pitch;
    else         
        x = xyzpIK(1); y = xyzpIK(2); z = xyzpIK(3); pitch = xyzpIK(4);
    end
    
    % print target
    fprintf('\nTarget IK end-effector position: x = %.2f, y = %.2f, z = %.2f, pitch = %.2f\n\n',x,y,z,pitch)
    
    % perform inverse kinematics
    result = inverseKinematics(x,y,z,pitch);
    
    % plot results
    for ii = 1:size(result,1)
        plotFK('Inverse Kinematics',result(ii,1),result(ii,2),result(ii,3),result(ii,4),result(ii,5),1,1)
    end
    
end

%% Task
if task_flag == 1 
    % perform task
    hold off; figure;
    pos = zeros(4,N*(size(xyzp,1)-1));
    kk = 1;
    [pos, kk] = task(xyzp,'Pick and Place Task',pos,kk);
    
end

%% Free Motion Trajectory
if free_motion_flag == 1
    hold off; figure;
%     xyzp = [0,300,0,-90;0,250,300,0;250,0,300,0;0,-250,300,0;0,-300,0,-90]; % positions/orientations
    posEEfree = freeMotion(xyzp,N);
end

%% Straight Line Trajectory
if straight_line_motion_flag == 1
    posEElin = straightLine(xyzp,N);
end

%% Cubic Polynomial Trajectory
if poly_motion_flag == 1    
    posEEpoly = polynomialTraj(xyzp,time_steps,N);
end

%% Quintic Polynomial Trajectory
if quintic_poly_motion_flag == 1
    posEEquint = quinticPolyTraj(xyzp,time_steps,N);
end

%% Linear with Parabolic Blend Trajectory
if lpbb_motion_flag == 1
%     time_steps = 10; % set number of time steps
%     xyzp = [0,300,0,-90;80,250,300,0;250,0,300,0;90,-250,300,0;0,-300,0,-90]; % positions/orientations
    parBlendTraj(xyzp,N);         
end

%% Obstacle Avoidance
if obstacle_avoidance_flag == 1
    xyzp = [0,300,0,-90;80,250,300,0;80,0,300,0;250,0,300,0;90,-250,300,0;0,-300,0,-90]; % positions/orientations
    posEEobs = obsAvoid(xyzp,N,time_steps);
end

%% plot trajectories
if  ~isempty(posEElin) && ~isempty(posEEfree) && ~isempty(posEEpoly) && ~isempty(posEEquint)
    hold off
    figure
    hold on
    title('Trajectories')
    plot3(posEElin(:,1),posEElin(:,2),posEElin(:,3),'*g-')
    plot3(posEEfree(:,1),posEEfree(:,2),posEEfree(:,3),'sr-')
    plot3(posEEpoly(:,1),posEEpoly(:,2),posEEpoly(:,3),'<b-')
    plot3(posEEquint(:,1),posEEquint(:,2),posEEquint(:,3),'dm-')
    xlabel('x');ylabel('y');zlabel('z');
    legend('Straight Line','Free Motion','Cubic Polynomial','Quintic Polynomial')
end

