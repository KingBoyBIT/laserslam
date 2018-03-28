function observe_heading(phi, useheading)
%function observe_heading(phi, useheading)
%
% Perform state update for a given heading measurement, phi,
% with fixed measurement noise: sigmaPhi
global XE PE

if useheading==0
    return;
end
sigmaPhi= 0.01*pi/180; % º½ÏòÆ«²îÖµradians, heading uncertainty

H= zeros(1,length(XE));
H(3)= 1;
v= pi_to_pi(phi - XE(3));

KF_cholesky_update(v, sigmaPhi^2, H);
