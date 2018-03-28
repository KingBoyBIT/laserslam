function C = Rover2G( x )
%UNTITLED2 Summary of this function goes here
%x(1:3)---------(x;y;z)
%x(4:6)---------(phix;phiy;phiz)
%   Detailed explanation goes here
C = [cos(x(5))*cos(x(6)) -cos(x(4))*sin(x(6))+sin(x(4))*sin(x(5))*cos(x(6))  sin(x(4))*sin(x(6))+cos(x(4))*sin(x(5))*cos(x(6)) x(1);
     cos(x(5))*sin(x(6))  cos(x(4))*cos(x(6))+sin(x(4))*sin(x(5))*sin(x(6)) -sin(x(4))*cos(x(6))+cos(x(4))*sin(x(5))*sin(x(6)) x(2);
        -sin(x(5))                        sin(x(4))*cos(x(5))                               cos(x(4))*cos(x(5))                x(3);
           0                                       0                                                0                           1 ];

end

