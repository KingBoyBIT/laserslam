function [z,idf]= get_observations(x, lm, idf, rmax,theta_range)
%function [z,idf]= get_observations(x, lm, idf, rmax)
%
% INPUTS:
%   x - vehicle pose [x;y;phi]
%   lm - set of all landmarks
%   idf - index tags for each landmark
%   rmax - maximum range of range-bearing sensor 
%   theta_range 激光测距仪转动臂转动角范围
%   L 激光测距仪转动臂长度
%   xl_R 体坐标系下激光测距仪安装位置
% OUTPUTS:
%   z - set of range-bearing observations
%   idf - landmark index tag for each observation
%  基于文章《Applying FastSLAM to Articulated Rovers》 fig.3.17编写
% Tim Bailey 2004.
global La xl_R
num_f = size(lm,2);
theta = zeros(1,num_f);
phi = zeros(1,num_f);
R = zeros(1,num_f);
C_RtoG = Rover2G(x);
% S_wtoG = omega2G(x);
for i = 1:num_f
    d_G = lm(:,i)-x(1:3);
    d_R = (C_RtoG(1:3,1:3))^-1*d_G;
    d_L = d_R-xl_R;
    h = sqrt((d_L(1))^2+(d_L(3))^2);
    if h>La
        theta(i) = pi_to_pi(pi/2-acos(La/h)+atan2(d_L(3),d_L(1)));
        dH_R = [d_L(1)-La*cos(theta(i)+pi/2);d_L(2);d_L(3)-La*sin(theta(i)+pi/2)];
    C_R2L = [cos(theta(i)) 0 -sin(theta(i));
                 0         1       0       ;
             sin(theta(i)) 0  cos(theta(i))];
    dH_L = C_R2L*dH_R;
        R(i) = norm(dH_R);
        phi(i) = pi_to_pi(atan2(dH_L(2),dH_L(1)));
    end
end

ii = find(theta>theta_range(1)&theta<theta_range(2)...
          &R<rmax&phi<2*pi/3&phi>-2*pi/3);
idf = idf(ii);
z = [R(ii);phi(ii);theta(ii)];

% sigmaR= 1; % metres
% sigmaB= (4.0*pi/180); % radians
% sigmaTHETA = (4.0*pi/180);
% z = z+[rand*sigmaR;rand*sigmaB;rand*sigmaTHETA];
end
    