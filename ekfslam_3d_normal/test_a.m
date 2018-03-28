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
V = 0.8;
w = 0.01;
dt = 0.2;
x = [100;20;0;0;0;0];
dxv = [];
x = constrain_terrain(x);
for i =1:4000
    C_RtoG = Rover2G(x(:,end));
    S_wtoG = omega2G(x(:,end));
    xv = x(:,end)+dt.*(blkdiag(C_RtoG(1:3,1:3),S_wtoG))*[V;0;0;0;0;w];
    xv = constrain_terrain(xv);
    dx = xv-x(:,end);
    dxv = [dxv,dx];
    x = [x,xv];
    i
end

% figure(2)
% i=4;

% surf(X(1:i:end,1:i:end),Y(1:i:end,1:i:end),Z(1:i:end,1:i:end)); hold on;
%      colormap(gray);
%      shading interp
%     axis([-100 100 0 200 -2 2]);
% plot3(path(1,:),path(2,:),path(3,:),'r','linewidth',2);
% Ap = cell(1,4);
% for j =1:4000
%     global X Y Z nx ny nz T_A2R r_w
% 
%     T_R2G= Rover2G(path(:,j)); %火星车到全局坐标的转换矩阵
%     z_A = zeros(4,1);
%     T_CtoA=cell(4,1); % 接触点到火星车轮的转化矩阵
%     for i=1:4
%         T_CtoG = T_R2G*T_A2R{i};
%         Ap{i}=[Ap{i},T_CtoG(1:3,4)-[0;0;r_w]]; 
%     end  
%     j
% end
% plot3(Ap{1}(1,:),Ap{1}(2,:),Ap{1}(3,:),'b--','linewidth',1);
% plot3(Ap{2}(1,:),Ap{2}(2,:),Ap{2}(3,:),'b--','linewidth',1);
% plot3(Ap{3}(1,:),Ap{3}(2,:),Ap{3}(3,:),'b--','linewidth',1);
% plot3(Ap{4}(1,:),Ap{4}(2,:),Ap{4}(3,:),'b--','linewidth',1);
