function S = omega2G(x)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%x(1:3)---------(x;y;z)
%x(4:6)---------(phix;phiy;phiz)
S = [1 sin(x(4))*tan(x(5)) cos(x(4))*tan(x(5));
     0        cos(x(4))         -sin(x(4))    ; 
     0 sin(x(4))/cos(x(5)) cos(x(4))/cos(x(5))];
end

