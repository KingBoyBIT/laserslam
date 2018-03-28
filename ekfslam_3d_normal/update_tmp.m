function obs_noise=update_tmp(z,R,idf,batch)
% function update(z,R,idf, batch)
%
% Inputs:
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   batch - switch to specify whether to process measurements together or sequentially
%
% Outputs:
%   XX, PX - updated state and covariance (global variables)

if batch == 1 % 成批处理观测值
    obs_noise=batch_update(z,R,idf);
else
    single_update(z,R,idf);
end

%
%
end
function obs_noise=batch_update(z,R,idf)
global XE

lenz= size(z,2);
lenx= length(XE);
H= zeros(3*lenz, lenx);
v= zeros(3*lenz, 1);
RR= zeros(3*lenz);
obs_noise=[0;0;0];
for i=1:lenz
    ii= 3*i + (-2:0);
    [zp,H(ii,:)]= observed_jac(XE, idf(i));
    
    v(ii)=      [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2));
        pi_to_pi(z(3,i)-zp(3))];
    RR(ii,ii)= R;
end
obs_noise(1)=mean(v(1:3:end));
obs_noise(2)=mean(v(2:3:end));
obs_noise(3)=mean(v(3:3:end));
KF_cholesky_update(v,RR,H);

%
%
end
function single_update(z,R,idf)
global XE

lenz= size(z,2);
for i=1:lenz
    [zp,H]= observe_model(XE, idf(i));
    
    v= [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    
    KF_cholesky_update(v,RR,H);
end        
end