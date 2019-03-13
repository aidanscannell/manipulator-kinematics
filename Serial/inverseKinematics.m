function solutions = inverseKinematics(x,y,z,pitch)
%	inverseKinematics -> Calculates arm angles for target end-effector
%	position (inverse kinematics)
%   
%   by Aidan Scannell
%
%	===== Inputs =====
%	x - target x position
%	y - target y position
%	z - target y position
%	pitch - target pitch

%	===== Outputs ====
%	solutions - array of armLink objects for each valid solution
global l1 l2 l3 l4
global range1 range2 range3 range4 range5

    %% calculate theta 1 (XY plane)
    theta1 = atan2d(y,x);

    %% calculate theta 3 (XZ plane)
    %calculate joint 4 coordinares
    x4 = sqrt(( x^2)+(y^2)) - (l4*cosd(pitch) );

    % x4 = x - l4 * cosd(pitch);
    z4 = z - l4 * sind(pitch) - l1;

    % calculate cos(theta3)
    c3 = (x4^2 + z4^2 - l2^2 - l3^2)/(2 * l2 * l3);
    
    if c3^2 > 1
        fprintf('Target position not in workspace\n')
        return
    end

    % calculate theta3
    theta3(1) = atan2d(sqrt(1-c3^2),c3);
    theta3(2) = atan2d(-sqrt(1-c3^2),c3);

    %% calculate theta 2 (XZ plane)
    % calculate cos(theta2) and sin(theta2)
    c2(1) = (x4*(l2 + l3*cosd(theta3(1))) + z4*l3*(sind(theta3(1))))/(x4^2 + z4^2);
    c2(2) = (x4*(l2 + l3*cosd(theta3(2))) + z4*l3*(sind(theta3(2))))/(x4^2 + z4^2);

    % calculate theta2
    theta2(1) = atan2d(sqrt(1-c2(1)^2),c2(1));
    theta2(2) = atan2d(sqrt(1-c2(2)^2),c2(2));
    theta2(3) = atan2d(-sqrt(1-c2(1)^2),c2(1));
    theta2(4) = atan2d(-sqrt(1-c2(2)^2),c2(2));

    %% calculate theta 4
%     theta4(1) = pitch - theta2(1) - theta3(1) - 90; % solution 1
%     theta4(2) = pitch - theta2(3) - theta3(1) - 90; % solution 2
%     theta4(3) = pitch - theta2(2) - theta3(2) - 90; % solution 3
%     theta4(4) = pitch - theta2(4) - theta3(2) - 90; % solution 4
    theta4(1) = pitch - theta2(1) - theta3(1); % solution 1
    theta4(2) = pitch - theta2(3) - theta3(1); % solution 2
    theta4(3) = pitch - theta2(2) - theta3(2); % solution 3
    theta4(4) = pitch - theta2(4) - theta3(2); % solution 4
    thetas = {[theta2(1), theta3(1), theta4(1)],[theta2(3), theta3(1), theta4(2)],...
        [theta2(2), theta3(2), theta4(3)],[theta2(4), theta3(2), theta4(4)]};
    
    %% determine correct solutions
    % perform FK for each solution and create a list of arm objects for each solution
    [link11 link21 link31 link41 link51] = forwardKinematics(theta1,theta2(1),theta3(1),theta4(1),0); 
    [link12 link22 link32 link42 link52] = forwardKinematics(theta1,theta2(3),theta3(1),theta4(2),0); 
    [link13 link23 link33 link43 link53] = forwardKinematics(theta1,theta2(2),theta3(2),theta4(3),0); 
    [link14 link24 link34 link44 link54] = forwardKinematics(theta1,theta2(4),theta3(2),theta4(4),0); 
    links = {[link11 link21 link31 link41 link51],[link12 link22 link32 link42 link52],...
        [link13 link23 link33 link43 link53],[link14 link24 link34 link44 link54]};
    
    % loop through solutions
    solutions = [];
    for i = 1:4
        pos4 = getP(links{i}(4)); % coordinates of joint 4
        pos5 = getP(links{i}(5)); % coordinates of joint 5

        % ensure joint angles are within joint ranges and that end-effector position is correct
        if round(pos5(1),2) == round(x,2) && round(pos5(2),2) == round(y,2) && round(pos5(3),2)...
                == round(z,2) && theta1 >= range1(1) && theta1 <= range1(2) && thetas{i}(1) >= range2(1) ...
                && thetas{i}(1)<= range2(2) && thetas{i}(2) >= range3(1) && thetas{i}(2) <= range3(2) && ...
                thetas{i}(3) >= range4(1) && thetas{i}(3) <= range4(2)

            % calculate pitch
            L_pitch = sqrt( (pos5(2) - pos4(2))^2 + (pos5(1) - pos4(1))^2 ); % calculate length of link 4-5 in XY plane
            FK_pitch = atan2d( (pos5(3) - pos4(3)), L_pitch ); % calculate pitch

            % print joint configuration
            fprintf('--------- INVERSE KINEMATICS SOLUTION %d ---------\n',i)
            fprintf('IK solution %d: theta 1 = %.2f, theta 2 = %.2f, theta 3 = %.2f, theta 4 = %.2f\n',i,theta1,thetas{i}(1),thetas{i}(2),thetas{i}(3))
            
            % print end-effector position and pitch
            fprintf('IK end-effector position for solution %d: x = %.2f, y = %.2f, z = %.2f, pitch = %.2f\n'...
                ,i,pos5(1),pos5(2),pos5(3),FK_pitch)
            
            solutions = [solutions; links{i}];
        end
    end

end