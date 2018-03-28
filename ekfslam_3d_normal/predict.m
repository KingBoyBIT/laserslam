function dz = predict (dxv, nxv,sigma_IMU)
%function predict (v,g,Q,WB,dt)
%
% Inputs:
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   XX, PX - predicted state and covariance (global variables)
%
% Tim Bailey 2004.
global XE PE 

% s= sin(g+XE(3)); 
% c= cos(g+XE(3));
% vts= v*dt*s; 
% vtc= v*dt*c;

% _____________jacobians______________________________%   
% Gv= [1 0 -vts;
%      0 1  vtc;
%      0 0 1];
% Gu= [dt*c -vts;
%      dt*s  vtc;
%      dt*tan(g)/WB v*dt/WB/(cos(g))^2];


  
% predict covariance
phix = XE(4);phiy = XE(5); phiz = XE(6);
C_RtoG = Rover2G(XE(1:6));
S_wtoG = omega2G(XE(1:6));
dx = blkdiag(C_RtoG(1:3,1:3),S_wtoG)*dxv;
dt=0.25;
G_IMU = blkdiag(C_RtoG(1:3,1:3),S_wtoG);
% Q_IMU = blkdiag(sigma_IMU(1),sigma_IMU(1),(sigma_IMU(2)),sigma_IMU(3),sigma_IMU(3),sigma_IMU(3));
Q_IMU = blkdiag(dt^2*sigma_IMU(1),dt^2*sigma_IMU(1),dt^2*(sigma_IMU(2)),dt*sigma_IMU(3),dt*sigma_IMU(3),dt*sigma_IMU(3));
jac_C2phix =[0 sin(phix)*sin(phiz)+cos(phix)*cos(phiz)*sin(phiy)  cos(phix)*sin(phiz)-cos(phiz)*sin(phix)*sin(phiy); 
             0 cos(phix)*sin(phiy)*sin(phiz)-cos(phiz)*sin(phix) -cos(phix)*cos(phiz)-sin(phix)*sin(phiy)*sin(phiz);
             0 cos(phix)*cos(phiy)                                                             -cos(phiy)*sin(phix)];
   
jac_C2phiy = [-cos(phiz)*sin(phiy) cos(phiy)*cos(phiz)*sin(phix) cos(phix)*cos(phiy)*cos(phiz);
              -sin(phiy)*sin(phiz) cos(phiy)*sin(phix)*sin(phiz) cos(phix)*cos(phiy)*sin(phiz);
              -cos(phiy)           -sin(phix)*sin(phiy)                   -cos(phix)*sin(phiy)];
jac_C2phiz = [-cos(phiy)*sin(phiz) -cos(phix)*cos(phiz)-sin(phix)*sin(phiy)*sin(phiz) cos(phiz)*sin(phix)-cos(phix)*sin(phiy)*sin(phiz);
              cos(phiy)*cos(phiz)  cos(phiz)*sin(phix)*sin(phiy)-cos(phix)*sin(phiz)  sin(phix)*sin(phiz) + cos(phix)*cos(phiz)*sin(phiy);
              0                    0                                                  0 ]; 
jac_S2phix = [0 cos(phix)*tan(phiy) -sin(phix)*tan(phiy);
               0 -sin(phix)          -cos(phix);
               0 cos(phix)/cos(phiy) -sin(phix)/cos(phiy)];
jac_S2phiy = [0 sin(phix)*(tan(phiy)^2 + 1)  cos(phix)*(tan(phiy)^2 + 1);
              0 0                               0;
              0 (sin(phix)*sin(phiy))/cos(phiy)^2 (cos(phix)*sin(phiy))/cos(phiy)^2]; 
G_x = blkdiag(eye(3),eye(3));
G_x(:,4) = [zeros(3,1);1;zeros(2,1)]+blkdiag(jac_C2phix,jac_S2phix)*dxv;
G_x(:,5) = [zeros(4,1);1;0]+blkdiag(jac_C2phiy,jac_S2phiy)*dxv;
G_x(:,6) = [zeros(5,1);1]+blkdiag(jac_C2phiz,zeros(3))*dxv;


PE(1:6,1:6) = G_x*PE(1:6,1:6)*G_x'+Q_IMU;
if size(PE,2)>6
    PE(1:6,6:end) = G_x*PE(1:6,6:end);
    PE(6:end,1:6) = (PE(1:6,6:end))';
end


dxn = G_IMU*nxv;
% predict state
XE(1:6) = XE(1:6)+dx+dxn;
dz = dx(3)+dxn(3);

% XE(1:3)= [XE(1) + vtc; 
%           XE(2) + vts;
%          pi_to_pi(XE(3)+ v*dt*sin(g)/WB)];
end