%% This is the simulation of generating craters with depth 
%% We generate three craters for the vehicle to extract 
%% The appeared craters on several images will be shown in file Measurement_Generation
clc; clear all;close all;
figure(1);
Roughness = 0.5;
D_size = 20;
use_Crater =1;
if use_Crater == 1
    Number_crater = 3;        % number of craters distributed on the planetary surface. 
    [X_range,Y_range] =... 
    meshgrid(linspace(0,200,D_size),linspace(0,200,D_size));
    Z_range = 0.*X_range-normrnd(0.1,Roughness,[D_size,D_size]);      % Add some roughness on the surface 
    
    surf(X_range,Y_range,Z_range); hold on;
    colormap([0 0 1]);
    %axis([-100 100 0 200 -50 50]);
    axis([0 200 0 200 -50 50]);
    
%     global Crater_map;                 %% Build the map of Crater
%     global target_spot;                %% Build the target landing spot
%     Crater_map(1,:) = [150,100];       %% Crater 1 position
%     Crater_map(2,:) = [400,400];       %% Crater 2 position
%     Crater_map(3,:) = [350,100];       %% Crater 3 position
%     Radius_Crater_1 = 60;              %% Crater 1 radius
%     Radius_Crater_2 = 70;              %% Crater 2 radius
%     Radius_Crater_3 = 80;              %% crater 3 radius
%     target_spot = [0 0];
%     draw_crater(Crater_map(1,1),Crater_map(1,2),Radius_Crater_1);
%     draw_crater(Crater_map(2,1),Crater_map(2,2),Radius_Crater_2);
%     draw_crater(Crater_map(3,1),Crater_map(3,2),Radius_Crater_3);
%     plot_crater(Crater_map(1,1),Crater_map(1,2),Radius_Crater_1);
%     plot_crater(Crater_map(2,1),Crater_map(2,2),Radius_Crater_2);
%     plot_crater(Crater_map(3,1),Crater_map(3,2),Radius_Crater_3);
%     plot_target(target_spot(1,1),target_spot(1,2),15);
% %     plot(Crater_map(1,1),Crater_map(1,2),'or','MarkerSize',15,'linewidth',1, 'MarkerFaceColor','r');
% %     text(Crater_map(1,1),Crater_map(1,2),20,'Crater One','linewidth',2);
% %     plot(Crater_map(2,1),Crater_map(2,2),'or','MarkerSize',15,'linewidth',1, 'MarkerFaceColor','r');
% %     plot(Crater_map(3,1),Crater_map(3,2),'or','MarkerSize',15,'linewidth',1, 'MarkerFaceColor','r');
% %     plot(Crater_map(3,1),Crater_map(3,2),'or','MarkerSize',15,'linewidth',1, 'MarkerFaceColor','r');
% %     plot(target_spot(1,1),target_spot(1,2),'oy','MarkerSize',18,'linewidth',1, 'MarkerFaceColor','y');
%     xlabel('Global Coornidate Axis X');
%     ylabel('Global Coornidate Axis Y');
%     zlabel('Global Coornidate Axis Z');
% hold on;
% 
% Q_t1 = [target_spot - Crater_map(1,:) 0]';
% Q_t2 = [target_spot - Crater_map(2,:) 0]';
% Q_t3 = [target_spot - Crater_map(3,:) 0]';
% 
% %     [X_range,Y_range] =...
% %     meshgrid(linspace(30,90,100),linspace(0,60,100));
% %     Z_range = 600 * ones(100,100);
% %     surf(X_range,Y_range,Z_range);
% %     colormap([0 0 1]);
end
[X,Y] =...
     meshgrid(linspace(0,200,400),linspace(0,200,400));
Z = interp2(X_range,Y_range,Z_range,X,Y,'spline');

figure(2)
surf(X,Y,Z); hold on;
     colormap(gray);
     shading interp
    %axis([-100 100 0 200 -2 2]);
    axis([0 200 0 200 -3 3]);
[nx,ny,nz]=surfnorm(X,Y,Z);
