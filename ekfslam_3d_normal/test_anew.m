% a = zeros(4,4);
% z_real = zeros(1,4);
% T_R2G= Rover2G(xtrue);
% for i=1:4
%     i
%     (T_A2R{i})^-1
%     a(i,:)=T_R2G*T_A2R{i}*[0;0;0;1];
%     z_real(i) = interp2(X,Y,Z,a(i,1),a(i,2));
% end
% 
% e = (a(:,3))'-z_real;
% dt = 0.25;
% x = [0;20;0;0;0;0];
% dxv = [];
% x = constrain_terrain(x);
% G= 0;
% V= 0.8; 
% MAXG= 50*pi/180; % 最大转角
% RATEG= 0.1; % 最大角速度
% WHEELBASE= 2; % 
% NUMBER_LOOPS = 2;
% AT_WAYPOINT= 1;
% iwp = 1;
% for i =1:4000
%     x2d = [x(1:2,end);x(6,end)];
%     [G,iwp]= compute_steering(x2d, waypoint, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
%     if iwp==0 & NUMBER_LOOPS > 1
%         pack; 
%         iwp=1; 
%         NUMBER_LOOPS= NUMBER_LOOPS-1; 
%     end
%     x2d= vehicle_model(x2d, V,G, WHEELBASE,dt);
%     x2d(3)=pi_to_pi(x2d(3));
%     C_RtoG = Rover2G(x(:,end));
%     S_wtoG = omega2G(x(:,end));
%     xv = x(:,end)+dt.*(blkdiag(C_RtoG(1:3,1:3),S_wtoG))*[V;0;0;0;0;w];
%     xv = [x2d(1:2);x(3,end);x(4:5,end);x2d(3)];
%     xv = constrain_terrain(xv);
%     dx = xv-x(:,end);
%     dxv = [dxv,dx];
%     x = [x,xv];
%     i
% end

figure(2)
i=8;

surf(X(1:i:end,1:i:end),Y(1:i:end,1:i:end),Z(1:i:end,1:i:end)); hold on;
     colormap(gray);
     shading interp
    axis([0 200 0 200 -3 3]);
plot3(path(1,:),path(2,:),path(3,:),'m','linewidth',2);
Ap = cell(1,4);
for j =1:4000
    global X Y Z nx ny nz T_A2R r_w

    T_R2G= Rover2G(path(:,j)); %火星车到全局坐标的转换矩阵
    z_A = zeros(4,1);
    T_CtoA=cell(4,1); % 接触点到火星车轮的转化矩阵
    for i=1:4
        T_CtoG = T_R2G*T_A2R{i};
        Ap{i}=[Ap{i},T_CtoG(1:3,4)-[0;0;r_w]]; 
    end  
    j
end
plot3(Ap{1}(1,:),Ap{1}(2,:),Ap{1}(3,:),'b--','linewidth',1);
plot3(Ap{2}(1,:),Ap{2}(2,:),Ap{2}(3,:),'b--','linewidth',1);
plot3(Ap{3}(1,:),Ap{3}(2,:),Ap{3}(3,:),'b--','linewidth',1);
plot3(Ap{4}(1,:),Ap{4}(2,:),Ap{4}(3,:),'b--','linewidth',1);
title('3D trajectory of the Mars rover with the terrain constraint')
xlabel('X_G')
ylabel('Y_G')
zlabel('Z_G')

