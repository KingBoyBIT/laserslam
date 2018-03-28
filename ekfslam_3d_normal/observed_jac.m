function [zp,H]= observed_jac(X, idf)
% 观测值预测量雅克比矩阵
% X为状态量
% idf 为预测特征点检索
% L为激光测距仪旋转臂长度
%  基于文章《Applying FastSLAM to Articulated Rovers》 fig.3.17及之后公式求解雅克比矩阵
global La xl_R
% Nxv= 3; % number of vehicle pose states
Nxv = 6;
% fpos= Nxv + idf*3 - 2; % position of xf in state
fpos= Nxv + idf*3 - 2; % position of xf in state
% fpos=  idf*Nxv - 2; % position of xf in state
% fpos= Nxv *(idf - 1)+1;
H= zeros(3, length(X));
zp = zeros(3,1);

xv=X(1);yv=X(2);zv=X(3);phix=X(4);phiy=X(5);phiz=X(6);
x=X(fpos);y=X(fpos+1);z=X(fpos+2);

C= [cos(phiy)*cos(phiz)                                                 cos(phiy)*sin(phiz)                        -sin(phiy);
    -cos(phix)*sin(phiz)+sin(phix)*sin(phiy)*cos(phiz) cos(phix)*cos(phiz)+sin(phix)*sin(phiy)*sin(phiz)  sin(phix)*cos(phiy);
    sin(phix)*sin(phiz)+cos(phix)*sin(phiy)*cos(phiz)  -sin(phix)*cos(phiz)+cos(phix)*sin(phiy)*sin(phiz) cos(phix)*cos(phiy)];
d_G = [x;y;z]-[xv;yv;zv]; 
d_R = C*d_G;
d_L = d_R-xl_R;
dx_L=d_L(1);
dz_L=d_L(3);
h = sqrt((dx_L)^2+(dz_L)^2);
if La<=h
theta = pi/2-acos(La/h)+atan2(dz_L,dx_L);
theta = pi_to_pi(theta);
dH_R = [d_L(1)-La*cos(theta+pi/2);d_L(2);d_L(3)-La*sin(theta+pi/2)];

C_R2L = [cos(theta) 0 -sin(theta);0  1  0;sin(theta) 0  cos(theta)];
dH_L = C_R2L*dH_R;

R = norm(dH_R);
phi = atan2(dH_L(2),dH_L(1));

dG_dpv = -eye(3);
dG_dxf = eye(3);

C_dphix =[0                                                   0                                                                     0;
          sin(phix)*sin(phiz)+cos(phix)*cos(phiz)*sin(phiy)   cos(phix)*sin(phiy)*sin(phiz)-cos(phiz)*sin(phix)   cos(phix)*cos(phiy);
          cos(phix)*sin(phiz)-cos(phiz)*sin(phix)*sin(phiy) -cos(phix)*cos(phiz)-sin(phix)*sin(phiy)*sin(phiz) -cos(phiy)*sin(phix)];
      
C_dphiy = [-cos(phiz)*sin(phiy)          -sin(phiy)*sin(phiz)                    -cos(phiy);         
           cos(phiy)*cos(phiz)*sin(phix) cos(phiy)*sin(phix)*sin(phiz) -sin(phix)*sin(phiy);
           cos(phix)*cos(phiy)*cos(phiz) cos(phix)*cos(phiy)*sin(phiz) -cos(phix)*sin(phiy)]; 
       
C_dphiz = [-cos(phiy)*sin(phiz)                               cos(phiy)*cos(phiz)                                 0;
           -cos(phix)*cos(phiz)-sin(phix)*sin(phiy)*sin(phiz) cos(phiz)*sin(phix)*sin(phiy)-cos(phix)*sin(phiz)   0;
           cos(phiz)*sin(phix)-cos(phix)*sin(phiy)*sin(phiz)  sin(phix)*sin(phiz) + cos(phix)*cos(phiz)*sin(phiy) 0];
dL_dpv = C*dG_dpv;
dL_dphix = C_dphix*d_G;
dL_dphiy = C_dphiy*d_G;
dL_dphiz = C_dphiz*d_G;
dL_dphi = [dL_dphix,dL_dphiy,dL_dphiz];
dLx_dxv = [dL_dpv(1,:),dL_dphi(1,:)];
dLy_dxv = [dL_dpv(2,:),dL_dphi(2,:)];
dLz_dxv = [dL_dpv(3,:),dL_dphi(3,:)];
dLx_dxf = C(1,:) *dG_dxf;
dLy_dxf = C(2,:) *dG_dxf;
dLz_dxf = C(3,:) *dG_dxf;


h_dxv = (1/h).*(dx_L.*dLx_dxv+dz_L.*dLz_dxv);
h_dxf = (1/h).*(dx_L.*dLx_dxf+dz_L.*dLz_dxf);

theta_dxv = -(1/(sqrt(1-(La/h)^2))).*La/h^2.*h_dxv+(1/(1+(dz_L/dx_L)^2)).*(dLz_dxv./dx_L-dLx_dxv.*dz_L./(dx_L)^2);
theta_dxf = -(1/(sqrt(1-(La/h)^2))).*La/h^2.*h_dxf+(1/(1+(dz_L/dx_L)^2)).*(dLz_dxf./dx_L-dLx_dxf.*dz_L./(dx_L)^2);

% theta_dxv = -(1/(sqrt(1-(La/h)^2))).*La/h^2.*h_dxv;
% theta_dxf = -(1/(sqrt(1-(La/h)^2))).*La/h^2.*h_dxf;
dHRx_dxv = dLx_dxv+La.*(sin(theta+pi/2)).*theta_dxv;
dHRy_dxv = dLy_dxv;
dHRz_dxv = dLz_dxv-La.*(cos(theta+pi/2)).*theta_dxv;
dHRx_dxf = dLx_dxf+La.*(sin(theta+pi/2)).*theta_dxf;
dHRy_dxf = dLy_dxf;
dHRz_dxf = dLz_dxf-La.*(cos(theta+pi/2)).*theta_dxf;
dHLx_dxv = dHRx_dxv.*cos(theta)-dH_L(1).*sin(theta).*theta_dxv-dHRz_dxv.*sin(theta)-dH_L(1).*cos(theta).*theta_dxv;
dHLy_dxv = dHRy_dxv ;
dHLx_dxf = dHRx_dxf.*cos(theta)-dH_L(1).*sin(theta).*theta_dxf-dHRz_dxf.*sin(theta)-dH_L(1).*cos(theta).*theta_dxf;
dHLy_dxf = dHRy_dxf;


R_dxv = (1/R).*((dH_R(1).*(dLx_dxv+La.*(sin(theta+pi/2))).*theta_dxv)+(dH_R(2).*dLy_dxv)+(dH_R(3).*(dLz_dxv-La.*(cos(theta+pi/2)).*theta_dxv)));
R_dxf = (1/R).*((dH_R(1).*(dLx_dxf+La.*(sin(theta+pi/2))).*theta_dxf)+(dH_R(2).*dLy_dxf)+(dH_R(3).*(dLz_dxf+La.*(cos(theta+pi/2)).*theta_dxf)));

phi_dxv = 1/(1+(dH_L(2)/dH_L(1))^2).*(dHLy_dxv./dH_L(1)-(dH_L(2)/(dH_L(1))^2).*dHLx_dxv);
phi_dxf = 1/(1+(dH_L(2)/dH_L(1))^2).*(dHLy_dxf./dH_L(1)-(dH_L(2)/(dH_L(1))^2).*dHLx_dxf);

zp= [R;phi;theta];
H(:,1:6)        = [R_dxv;phi_dxv;theta_dxv];
H(:,fpos:fpos+2)= [R_dxf;phi_dxf;theta_dxf];
end
end
 