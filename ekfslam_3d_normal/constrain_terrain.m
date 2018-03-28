function x = constrain_terrain( xv )
% xv-û�м������Լ��ǰ�Ļ��ǳ�״̬
% x �������Լ����Ļ��ǳ�״̬
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global X Y Z nx ny nz T_A2R r_w

T_R2G= Rover2G(xv); %���ǳ���ȫ�������ת������
z_A = zeros(4,1);
T_CtoA=cell(4,1); % �Ӵ��㵽���ǳ��ֵ�ת������
for i=1:4
    a=T_R2G*T_A2R{i}*[0;0;0;1];
    fax = interp2(X,Y,nx,a(1),a(2));
    fay = interp2(X,Y,ny,a(1),a(2));
    faz = interp2(X,Y,nz,a(1),a(2));
    fa=[fax fay faz];
    z= [0 0 1];
    delta = acos(dot(fa,z)/(norm(fa)*norm(z)));
    z_A(i) = interp2(X,Y,Z,a(1),a(2));
    T_CtoA{i} = [cos(delta) 0 sin(delta) -r_w*sin(delta);
                    0       1      0           0;
                 -sin(delta) 0 cos(delta) -r_w*cos(delta);
                    0        0     0            1         ];
end

xx =  fminsearch(@(xx)z_erros( xx,xv,z_A,T_CtoA ),[0;0;0]);
x = xv;
x(3:5)=xx;
end

function e = z_erros( xx,xv,z_A,T_CtoA )
global T_A2R
z = xx(1);
phix = xx(2);
phiy = xx(3);

T_RtoG = Rover2G([xv(1:2);z;phix;phiy;xv(end)]);

e_z=zeros(4,1);

for i=1:4
    T_CtoG = T_RtoG*T_A2R{i}*T_CtoA{i};
    e_z(i) = T_CtoG(3,4)-z_A(i);
end

e = e_z'*e_z;
end
