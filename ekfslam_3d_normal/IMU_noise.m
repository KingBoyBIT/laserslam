function nxv = IMU_noise(sigma,t)
%function 
%产生固联IMU的随机漂移噪声，并积分到状态量
% Add random noise to nominal control values. We assume Q is diagonal.
p_noise = normrnd(0,sqrt(sigma(1)),2,1);
phi_noise = normrnd(0,sqrt(sigma(3)),1,1);
pz_noise = normrnd(0,sqrt(sigma(2)),1,1);

nxv(1:2) = p_noise.*t^2;
nxv(3) = pz_noise.*t^2;
nxv(5) = 1.*phi_noise.*t;
nxv(4) = 1.*phi_noise.*t;
nxv(6) = phi_noise.*t;
% xv= normrnd(0,1,6,1);
% dx = 0*dx;
% nxv(4:6) = [dx(4)*xv(4),dx(5)*xv(5),dx(6)*xv(6)];
nxv=nxv';
end

