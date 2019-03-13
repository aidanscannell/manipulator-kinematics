%	Arm class for planar parallell robot. 
%	Robotic Fundamentals UFMF4X-15-M
%   arm.m
%   
%   by Aidan Scannell
classdef arm < handle
    properties
        theta % link 1 angle
        psi % link 2 angle
        PBPP % vector from PB to PP
        CPP % distance from C to PP
        BPB % vector between centre of base platform and base joint for arm i
        PB % x,y position of base 
        points % matrix of [x,y] positions for each joint of arm
        T % transformation matrix
    end
    methods
        function obj = arm(i,R_BC,BC,s_a,r_p,L,r_b,a,X_c,Y_c)
            if nargin > 0
                % set a and PB for joint i
                if i == 1
                    a = a + 30;
                    obj.PB(1) = 0;
                    obj.PB(2) = 0;
%                     BC = [r_b*cosd(30)-X_c;r_b*sind(30)-Y_c;0]; % Define vector BC
                elseif i == 2
                    a = a + 120 + 30;
                    obj.PB(1) = 2*r_b*cosd(30);
                    obj.PB(2) = 0;
%                     BC = [X_c-r_b*cosd(30);Y_c-r_b*sind(30);0]; % Define vector BC
                elseif i == 3
                    a = a + 120*2 + 30;
                    obj.PB(1) = r_b*cosd(30);
                    obj.PB(2) = r_b*sind(30) + r_b;
%                     BC = [X_c-r_b*cosd(30);Y_c-r_b*sind(30);0]; % Define vector BC
                end   
                obj.BPB = [-r_b*cosd(a); -r_b*sind(a); 0];
                obj.CPP = [-r_p*cosd(a); -r_p*sind(a); 0];
                obj.PBPP = R_BC * obj.CPP + BC - obj.BPB; % [xpp, ypp, 0]

                e_1 = -2 * obj.PBPP(2) * s_a;
                e_2 = -2 * obj.PBPP(1) * s_a;
                e_3 = (obj.PBPP(1))^2 + (obj.PBPP(2))^2 + s_a^2 - L^2;

                t_1 = ( -e_1 + sqrt(e_1^2 + e_2^2 - e_3^2) ) / (e_3 - e_2);
                t_2 = ( -e_1 - sqrt(e_1^2 + e_2^2 - e_3^2) ) / (e_3 - e_2);        
                
                obj.T = [R_BC(1,1), R_BC(1,2), R_BC(1,3), X_c;
                         R_BC(2,1), R_BC(2,2), R_BC(2,3), Y_c;
                         R_BC(3,1), R_BC(3,2), R_BC(3,3), 0;
                         0,         0,         0,         1];
                
                if t_1 == real(t_1)
                    obj.theta(1) = 2*atand(t_1);
                    obj.theta(2) = 2*atand(t_2);

                    c_psi = ( obj.PBPP(1) - s_a*cosd(obj.theta(1)) ) / L;
                    s_psi = ( obj.PBPP(2) - s_a*sind(obj.theta(1)) ) / L;

                    obj.psi = atan2d(s_psi,c_psi);
                else
                    obj.theta(1) = NaN;
                    obj.theta(2) = NaN;
                    obj.psi = NaN;                    
                end
            end
        end
        function theta = getTheta(obj)
            theta = obj.theta;
        end
        function psi = getPsi(obj)
            psi = obj.psi;
        end
        function points = getJointCo(obj,s_a,L)          
            points = [[obj.PB(1), obj.PB(1) + s_a*cosd(obj.theta(1)), obj.PB(1) + s_a*cosd(obj.theta(1)) + L*cosd(obj.psi)]; [obj.PB(2), obj.PB(2) + s_a*sind(obj.theta(1)), obj.PB(2) + s_a*sind(obj.theta(1)) + L*sind(obj.psi)]];
            obj.points = points;
        end
    end
end
