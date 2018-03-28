function p = transformtoglobal(veh, b)
% function p = TransformToGlobal(p, b)
%
% Transform a list of poses [x;y;phi] so that they are global wrt a base pose
%
% Tim Bailey 1999

% rotate
veh = veh(1:3,:);
rot = Rover2G(b);
veh = [veh;ones(1,4)];
p=rot*veh;
p = p(1:3,:);
% if p is a pose and not a point
if size(veh,1)==6
   veh(4:6,:) = pi_to_pi(veh(4:6,:) + b(4:6));
end
