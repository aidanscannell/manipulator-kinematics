%	Arm Link class for lynxmotion robotic arm. 
%	Robotic Fundamentals UFMF4X-15-M
%   armLink.m
%   
%   by Aidan Scannell
classdef armLink < handle
    properties
        T % transformation matrix wrt previous link
        To % transformation matrix wrt to origin        
        a_n_1 % link length, the distance from z(i?1) to z(i) measured along x(i?1)
        alpha_n_1 % twist angle, the angle between z(i?1) to z(i) measured about x(i?1)
        d % offset length, the distance from x(i?1) to x(i) measured along z(i)
        theta % Joint angle, the angle between x(i?1) to x(i) measured about z(i)
        p % 3D coordinates of the end of the link
        i % link number
    end
    methods
        function obj = armLink(a_n_1,alpha_n_1,d,theta,i)
          if nargin > 0
              if i == 4
                  obj.theta = theta + 90;
              else
                  obj.theta = theta;
              end
              obj.a_n_1 = a_n_1;
              obj.alpha_n_1 = alpha_n_1;
              obj.d = d; 
              obj.T = [ cosd(theta)                 -sind(theta)                 0                a_n_1;
                       sind(theta)*cosd(alpha_n_1)  cosd(theta)*cosd(alpha_n_1)   -sind(alpha_n_1)  -sind(alpha_n_1)*d;
                       sind(theta)*sind(alpha_n_1)  cosd(theta)*sind(alpha_n_1)   cosd(alpha_n_1)   cosd(alpha_n_1)*d;
                       0                          0                           0                1];
          end
        end
        function calcTo(obj,To_prev)
            if To_prev == 0 
                obj.To = obj.T;
            else
                obj.To = To_prev * obj.T;
            end
        end
        function p = getP(obj)
            obj.p = obj.To(1:3,4)';
            p = obj.p;
        end
        function theta = getTheta(obj)
            theta = obj.theta;
        end
    end
end
