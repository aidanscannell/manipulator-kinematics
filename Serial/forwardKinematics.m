function [link1 link2 link3 link4 link5] = forwardKinematics(q1,q2,q3,q4,q5)
%	forwardKinematics -> Creates armLink objects for each arm - perform forward kinematics 
%   
%   by Aidan Scannell
%
%	===== Inputs =====
%	qN - angle of joint N

%	===== Outputs ====
%	linkN - armLink object for link N
global l1 l2 l3 l4

    %% Initialise arm links
    link1 = armLink(0,0,l1,q1,1);
    link2 = armLink(0,90,0,q2,2);
    link3 = armLink(l2,0,0,q3,3);
    link4 = armLink(l3,0,0,q4-90,4);
    link5 = armLink(0,-90,l4,q5,5);

    %% Calculate compound transformation matrices
    link1.calcTo(0);
    link2.calcTo(link1.To);
    link3.calcTo(link2.To);
    link4.calcTo(link3.To);
    link5.calcTo(link4.To);

end