syms xv yv zv phix phiy phiz x y z L dx_L dz_L

C_RtoG =     [cos(phiy)*cos(phiz) -cos(phix)*sin(phiz)+sin(phix)*sin(phiy)*cos(phiz)  sin(phix)*sin(phiz)+cos(phix)*sin(phiy)*cos(phiz) ;
              cos(phiy)*sin(phiz)  cos(phix)*cos(phiz)+sin(phix)*sin(phiy)*sin(phiz) -sin(phix)*cos(phiz)+cos(phix)*sin(phiy)*sin(phiz) ;
                    -sin(phiy)                        sin(phix)*cos(phiy)                               cos(phix)*cos(phiy)             ];
C= [cos(phiy)*cos(phiz)                                                 cos(phiy)*sin(phiz)                        -sin(phiy);
    -cos(phiy)*sin(phiz)+sin(phix)*sin(phiy)*cos(phiz) cos(phix)*cos(phiz)+sin(phix)*sin(phiy)*sin(phiz)  sin(phix)*cos(phiy);
    sin(phix)*sin(phiz)+cos(phix)*sin(phiy)*cos(phiz)  -sin(phix)*cos(phiz)+cos(phix)*sin(phiy)*sin(phiz) cos(phix)*cos(phiy)];
d_G = [x;y;z]-[xv;yv;zv];
d_R = C*d_G;
d_L = d_R-[0.75;0.5;0];
dx_L=d_L(1);
dz_L=d_L(3);
h = sqrt((dx_L)^2+(dz_L)^2);
theta = pi/2-acos(L/h)-atan2(dz_L,dx_L);
dH_R = [d_L(1)-L*cos(theta+pi/2);d_L(2);d_L(3)-L*sin(theta+pi/2)];
% C_R2L = [cos(theta) 0 -sin(theta);
%             0         1       0  ;
%          sin(theta) 0  cos(theta)];
% dH_L = C_R2L*dH_R;
R = norm(dH_R);
phi = atan2(dH_R(2),dH_R(1));
f = [R;phi;theta];

% jacobian(f,[xv yv zv phix phiy phiz]);
jacobian(C,phiz);
