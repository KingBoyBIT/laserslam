function lm = build_map(X,Y,Z,num_feature)
alpha=-pi/2:2*pi/50:2*pi-pi/2;
R=80; 
x=R*cos(alpha)+100;
y=R*sin(alpha)+R+10;
z=interp2(X,Y,Z,x,y,'spline');

surf(X,Y,Z); hold on;
colormap(gray);
shading interp
axis([0 200 0 200 -3 3]);

for i=1:length(x)
    plot3(x(i),y(i),z(i),'b.');
    hold on
end

wp = [x;y;z];
% num_feature = 200;
lm = [];
lm(1,:) =  200.*rand(1,num_feature);
lm(2,:) =  200.*rand(1,num_feature);
lm(3,:)=interp2(X,Y,Z,lm(1,:),lm(2,:));



for j=1:num_feature
    plot3(lm(1,j),lm(2,j),lm(3,j),'g*');
    axis square
end
end