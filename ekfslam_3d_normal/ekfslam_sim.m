function [data,dxz,XE_out,PE_out,da_table]= ekfslam_sim(lm,t)
%function data= ekfslam_sim(lm, wp)
%
% INPUTS:
%   lm - set of landmarks
%   wp - set of waypoints
%
% OUTPUTS:
%   data - a data structure containing:
%       data.true: the vehicle 'true'-path (ie, where the vehicle *actually* went)
%       data.path: the vehicle path estimate (ie, where SLAM estimates the vehicle went)
%       data.state(k).x: the SLAM state vector at time k
%       data.state(k).P: the diagonals of the SLAM covariance matrix at time k
%
% NOTES:
%   This program is a SLAM simulator. To use, create a set of landmarks and
%   vehicle waypoints (ie, waypoints for the desired vehicle path). The program
%   'frontend.m' may be used to create this simulated environment - type
%   'help frontend' for more information.
%       The configuration of the simulator is managed by the script file
%   'configfile.m'. To alter the parameters of the vehicle, sensors, etc
%   adjust this file. There are also several switches that control certain
%   filter options.
%
% Tim Bailey and Juan Nieto 2004.
% Version 2.0

% clear,clc,close all
format compact
configfile; % ** 初始化参数
% main
close all
% 建立演示窗口
fig=figure(2);
% 显示地图表面
rng(20)
% surf(X,Y,Z); hold on;
% colormap(gray);
% shading interp
axis([0 200 0 200 -3 3]);

% 显示路标
plot(lm(1,:),lm(2,:),'g*');
hold on
axis tight
%plot(wp(1,:),wp(2,:), 'r', wp(1,:),wp(2,:),'b.');
xlabel('metres'), ylabel('metres');zlabel('metres');
set(fig, 'name', 'EKF-SLAM Simulator');
h= setup_animations;
veh= [2 2 -2 -2;1 -1 -1 1;0 0 0 0]; % vehicle animation
plines=[]; % for laser line animation
pcount=0;
dxz=[];
% Initialise states and other global variables
global XE PE DATA
xtrue= path(:,1);
XE= xtrue;
PE=  blkdiag(0.00*eye(3),0.00*eye(3));
DATA= initialise_store(XE,PE,XE,[0;0;0]); % 存储初始参数
% Initialise other variables and constants
dt= DT_CONTROLS;        % 仿真控制周期
dtsum= 0;               % change in time since last observation
ftag= 1:size(lm,2);     % identifier for each landmark
da_table= zeros(1,size(lm,2)); % 数据关联表
G= 0;                   % 运动舵轮角度
% QE= Q;
RE= R;

if SWITCH_INFLATE_NOISE %是否增大噪声
% 	QE= 0*Q; RE= 0*R;
% 	QE= 1.5*Q; RE= 1.5*R;
% 	QE= 1.5*Q; RE= 2*R;
	QE= Q; RE= R;
end % inflate estimated noises (ie, add stabilising noise)

if SWITCH_SEED_RANDOM  % 加入初始误差
	rand('state',SWITCH_SEED_RANDOM);
	randn('state',SWITCH_SEED_RANDOM);
end

if SWITCH_PROFILE
	profile on -detail builtin;
end

% t = 600;  %仿真时间
n_step=1;

% Main loop
while dt*n_step<t
	n_step = n_step+1;
	% 仿真计算画出轨迹对应的控制量V和G
	%     [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
	%     if iwp==0 & NUMBER_LOOPS > 1
	%         pack;
	%         iwp=1;
	%         NUMBER_LOOPS= NUMBER_LOOPS-1;
	%     end % perform loops: if final waypoint reached, go back to first
	%     C = Rover2G(path(:,n_step-1));
	%     S = omega2G(path(:,n_step-1));
	C = Rover2G(xtrue);
	S = omega2G(xtrue);
	dxn = dxv(:,n_step-1);
	xtrue = xtrue+dxn;
	dxn = blkdiag(inv(C(1:3,1:3)),inv(S))*dxn;
	nxv= IMU_noise(sigma_IMU,dt);
	
	% EKF 预测过程 predict step
	dz=predict (dxn, nxv,sigma_IMU);
	dxz = [dxz dz];
	% @@@@@If heading known, observe heading
	observe_heading(xtrue(3), SWITCH_HEADING_KNOWN);
	
	% 测量过程, (available every DT_OBSERVE seconds)
	dtsum= dtsum + dt;
	obs_noise = 0*ones(3,1);
	if dtsum >= DT_OBSERVE
		dtsum= 0;
		
		% 计算真实测量值
		[z,ftag_visible]= get_observations(xtrue, lm, ftag, MAX_RANGE,THETA_RANGE);
		if ~isempty(z)
			z = add_observation_noise(z,R, SWITCH_SENSOR_NOISE); %加入测量噪声
		end

		% EKF update step
		if SWITCH_ASSOCIATION_KNOWN == 1   % 在数据关联已知或者未知下确定新路标以及已存在路标
			[zf,idf,zn, da_table]= data_associate_known(XE,z,ftag_visible, da_table);
		else
			[zf,idf, zn]= data_associate(XE,PE,z,RE, GATE_REJECT, GATE_AUGMENT);
		end
		
		if SWITCH_USE_IEKF == 1
			update_iekf(zf,RE,idf, 5);
		elseif ~isempty(zf)
			obs_noise=update_tmp(zf,RE,idf,SWITCH_BATCH_UPDATE);
		end
		if ~isempty(zn)
			augment(zn,RE);
		end
	end
	
	
	% data store (offline)
	store_data(XE, PE, xtrue,obs_noise);
	
	% Plot
	xt= transformtoglobal(veh, xtrue);
	set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:),'zdata', xt(3,:))
	
	if SWITCH_GRAPHICS
		xv= transformtoglobal(veh, XE(1:6));
		pvcov= make_vehicle_covariance_ellipse(XE,PE);
		set(h.xv, 'xdata', xv(1,:), 'ydata', xv(2,:)','zdata', xv(3,:))
		set(h.vcov, 'xdata', pvcov(1,:), 'ydata', pvcov(2,:),'zdata', pvcov(3,:))
		
		pcount= pcount+1;
		if pcount == 200 % plot path infrequently
			pcount=0;
			set(h.pth, 'xdata', DATA.path(1,1:DATA.i), 'ydata', DATA.path(2,1:DATA.i), 'zdata', DATA.path(3,1:DATA.i));
			set(h.ture_pth, 'xdata', DATA.true(1,1:DATA.i), 'ydata', DATA.true(2,1:DATA.i),'zdata', DATA.true(3,1:DATA.i)) ;
		end
		
		if dtsum==0 && ~isempty(z) % plots related to observations
			set(h.xf, 'xdata', XE(7:3:end), 'ydata', XE(8:3:end),'ydata', XE(8:3:end))
			plines= make_laser_lines (z,XE(1:3));
			set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:),'zdata', plines(3,:))
			pfcov= make_feature_covariance_ellipses(XE,PE);
			set(h.fcov, 'xdata', pfcov(1,:), 'ydata', pfcov(2,:))
		end
	end
	drawnow
end
% end of main loop

if SWITCH_PROFILE
	profile
	report
end

data = finalise_data(DATA);
set(h.pth, 'xdata', data.path(1,:), 'ydata', data.path(2,:))
XE_out = XE;
PE_out = PE;
clear global DATA
clear global XX
clear global PX


%
%
end
function h= setup_animations()
% h.xt= patch(0,0,'b','erasemode','xor'); % vehicle true
% h.xv= patch(0,0,'r','erasemode','xor'); % vehicle estimate
% h.pth= plot(0,0,'k--','markersize',2,'erasemode','background','linewidth',1); % vehicle path estimate
% h.ture_pth= plot(0,0,'r','markersize',2,'erasemode','background','linewidth',2); % vehicle path estimate
% h.obs= plot(0,0,'b','erasemode','xor'); % observations
% h.xf= plot(0,0,'r+','erasemode','xor'); % estimated features
% h.vcov= plot(0,0,'b','erasemode','xor'); % vehicle covariance ellipses
% h.fcov= plot(0,0,'b','erasemode','xor'); % feature covariance ellipses
h.xt= patch(0,0,'b'); % vehicle true
h.xv= patch(0,0,'r'); % vehicle estimate
h.pth= plot(0,0,'k--','markersize',2,'linewidth',1); % vehicle path estimate
h.ture_pth= plot(0,0,'r','markersize',2,'linewidth',2); % vehicle path estimate
h.obs= plot(0,0,'b'); % observations
h.xf= plot(0,0,'r+'); % estimated features
h.vcov= plot(0,0,'b'); % vehicle covariance ellipses
h.fcov= plot(0,0,'b'); % feature covariance ellipses

end
%
%

function p= make_laser_lines (rb,xv)
% compute set of line segments for laser range-bearing measurements
if isempty(rb), p=[];
	return,
end

global XE  xl_R La

r= rb(1); phi= rb(2);theta =rb(3);
xf_L = r*cos(phi);
yf_L = r*sin(phi);
zf_L = La;
T_L2R = [[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)],xl_R;0 0 0 1]; %激光测距仪坐标系转化到本体坐标系
T_R2G = Rover2G(XE(1:6));  %本体坐标系转化到惯性系
pf_G = T_R2G*T_L2R*[xf_L;yf_L;zf_L;1];

len= size(rb,2);
lnes(1,:)= zeros(1,len)+ xv(1);
lnes(2,:)= zeros(1,len)+ xv(2);
lnes(3,:)= zeros(1,len)+ xv(3);
lnes(4:6,:)= [pf_G(1).*ones(1,len);pf_G(2).*ones(1,len);pf_G(3).*ones(1,len)];
p= line_plot_conversion (lnes);
end
%
%

function p= make_vehicle_covariance_ellipse(x,P)
% compute ellipses for plotting vehicle covariances
N= 20;
inc= 2*pi/N;
phi= 0:inc:2*pi;
circ= 2*[cos(phi); sin(phi)];

p= make_ellipse(x(1:3), P(1:2,1:2), circ);
end
function p= make_feature_covariance_ellipses(x,P)
% compute ellipses for plotting feature covariances
N= 20;
inc= 2*pi/N;
phi= 0:inc:2*pi;
circ= 2*[cos(phi); sin(phi)];

lenx= length(x);
lenf= (lenx-6)/3;
p= zeros (3, lenf*(N+2));

ctr= 1;
for i=1:lenf
	ii= ctr:(ctr+N+1);
	jj= 4+3*i; jj= jj:jj+2;
	
	p(:,ii)= make_ellipse(x(jj), P(jj(1:2),jj(1:2)), circ);
	ctr= ctr+N+2;
end
end
%
%

function p= make_ellipse(x,P,circ)
% make a single 2-D ellipse
% r= sqrtm_2by2(P);
r = diag(sqrt(diag(P)));
k = 2; %画图时的放大系数，方便与偏差椭圆的显示

a= k.*r*circ;
p(2,:)= [a(2,:)+x(2) NaN];
p(1,:)= [a(1,:)+x(1) NaN];
p(3,:)= [x(3).*ones(1,size(circ,2)) NaN];
end
%
%

function data= initialise_store(x,P, xtrue,noise)
% offline storage initialisation
data.i=1;
data.path= x;
data.true= xtrue;
data.obs_noise = noise;
data.state(1).x= x;
%data.state(1).P= P;
data.state(1).P= diag(P);
end
%
%

function store_data(x, P, xtrue,noise)
% add current data to offline storage
global DATA
CHUNK= 5000;
len= size(DATA.path,2);
if DATA.i == len % grow array exponentially to amortise reallocation
	if len < CHUNK, len= CHUNK; end
	DATA.path= [DATA.path zeros(6,len)];
	DATA.true= [DATA.true zeros(6,len)];
	DATA.obs_noise = [DATA.obs_noise zeros(3,len)];
	pack
end
i= DATA.i + 1;
DATA.i= i;
DATA.path(:,i)= x(1:6);
DATA.true(:,i)= xtrue;
DATA.obs_noise(:,i) = noise;
DATA.state(i).x= x;
%DATA.state(i).P= P;
DATA.state(i).P= diag(P);
end
%
%

function data = finalise_data(data)
% offline storage finalisation
data.path= data.path(:,1:data.i);
data.true= data.true(:,1:data.i);
data.obs_noise = data.obs_noise(:,1:data.i);
end