function augment(z,R)
%function augment(z,R)
%
% Inputs:
%   z, R - range-bearing measurements and covariances, each of a new feature
%
% Outputs:
%   x, P - augmented SLAM state and covariance (global variables)
%
% Notes: 
%   - We assume the number of vehicle pose states is three.
%   - Only one value for R is used, a s all measurements are assumed to 
%   have same noise properties.
%
% Tim Bailey 2004.

% add new features to state
for i=1:size(z,2)
    add_one_z(z(:,i),R);
end

%
%

function add_one_z(z,R)
global XE PE xl_R La

len= length(XE);
r= z(1); phi= z(2);theta =z(3);
xf_L = r*cos(phi);
yf_L = r*sin(phi);
zf_L = La;
T_L2R = [[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)],xl_R;0 0 0 1]; %激光测距仪坐标系转化到本体坐标系
T_R2G = Rover2G(XE(1:6));  %本体坐标系转化到惯性系
pf_G = T_R2G*T_L2R*[xf_L;yf_L;zf_L;1];
phix = XE(4);phiy = XE(5);phiz=XE(6);
% 扩展状态
XE = [XE;
      pf_G(1:3)];

% % augment x
% XE= [XE;
%      XE(1) + r*c;
%      XE(2) + r*s];

% 求初始化雅克比矩阵
g_dpv = eye(3);
% 本体系到全局系转移矩阵对phix，phiy，phiz雅克比矩阵
Tr2g_dphix = [0 sin(phix)*sin(phiz)+cos(phix)*cos(phiz)*sin(phiy)   cos(phix)*sin(phiz)-cos(phiz)*sin(phix)*sin(phiy)   0;
              0 cos(phix)*sin(phiy)*sin(phiz)-cos(phiz)*sin(phix)   -cos(phix)*cos(phiz)-sin(phix)*sin(phiy)*sin(phiz)  0;
              0                cos(phix)*cos(phiy)                              -cos(phiy)*sin(phix)                    0;
              0                          0                                              0                               0];
Tr2g_dphiy = [-cos(phiz)*sin(phiy)  cos(phiy)*cos(phiz)*sin(phix) cos(phix)*cos(phiy)*cos(phiz) 0;
              -sin(phiy)*sin(phiz)  cos(phiy)*sin(phix)*sin(phiz) cos(phix)*cos(phiy)*sin(phiz) 0;
              -cos(phiy)                 -sin(phix)*sin(phiy)           -cos(phix)*sin(phiy)    0;
                  0                              0                             0                0];
Tr2g_dphiz = [-cos(phiy)*sin(phiz) -cos(phix)*cos(phiz)-sin(phix)*sin(phiy)*sin(phiz)  cos(phiz)*sin(phix)-cos(phix)*sin(phiy)*sin(phiz) 0;
              cos(phiy)*cos(phiz)  cos(phiz)*sin(phix)*sin(phiy)-cos(phix)*sin(phiz)   sin(phix)*sin(phiz)+cos(phix)*cos(phiz)*sin(phiy) 0;
                      0                                     0                                      0                                     0;
                      0                                     0                                      0                                     0];
g_dphix = Tr2g_dphix*T_L2R*[xf_L;yf_L;zf_L;1];
g_dphiy = Tr2g_dphiy*T_L2R*[xf_L;yf_L;zf_L;1];
g_dphiz = Tr2g_dphiz*T_L2R*[xf_L;yf_L;zf_L;1];

Tl2r_dtheta = [-sin(theta) 0 cos(theta)  0;
                   0       0      0      0;
               -cos(theta) 0 -sin(theta) 0;
                   0       0      0      0];
g_dr = T_R2G*T_L2R*[cos(phi);sin(phi);0;0];
g_dphi = T_R2G*T_L2R*[-r*sin(phi);r*cos(phi);0;0];
g_dtheta = T_R2G*Tl2r_dtheta*[xf_L;yf_L;zf_L;1];

Gv = [g_dpv,g_dphix(1:3),g_dphiy(1:3),g_dphiz(1:3)];
Gz = [g_dr(1:3),g_dphi(1:3),g_dtheta(1:3)];
                             
% Gv= [1 0 -r*s;
%      0 1  r*c];
% Gz= [c -r*s;
%      s  r*c];
     
% augment P
rng= len+1:len+3;
PE(rng,rng)= Gv*PE(1:6,1:6)*Gv' + Gz*R*Gz'; % feature cov
PE(rng,1:6)= Gv*PE(1:6,1:6); % vehicle to feature xcorr
PE(1:6,rng)= PE(rng,1:6)';
if len>6
    rnm= 6:len;
    PE(rng,rnm)= Gv*PE(1:6,rnm); % map to feature xcorr
    PE(rnm,rng)= PE(rng,rnm)';
end
