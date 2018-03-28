function z= add_observation_noise(z,R, addnoise)
%function z= add_observation_noise(z,R, addnoise)
%
% Add random measurement noise. We assume R is diagonal.
if addnoise == 1
    len= size(z,2);
    if len > 0
        noise = zeros(2,len);
        noise(1,:)=sqrt(R(1,1)).*randn(1,len);
        noise(2,:)=sqrt(R(2,2)).*randn(1,len);
        noise(3,:)=sqrt(R(3,3)).*randn(1,len);
        z(1,:)= z(1,:) + noise(1,:);
        z(2,:)= z(2,:) + noise(2,:);
        z(3,:)= z(3,:) + noise(3,:);
    end
end
