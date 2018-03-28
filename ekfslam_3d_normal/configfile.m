%%% Configuration file
%%% Permits various adjustments to parameters of the SLAM algorithm.
%%% See ekfslam_sim.m for more information
% ģ����ά���β�����������
load('test_terrain0322.mat');
load('path0322.mat');
global X Y Z nx ny nz r_w
%����������
V= 0.8; % m/s
r_w = 0.2;%���ǳ��ְ뾶
omega = 0.01;%���ٶȣ�rad/s
% MAXG= 30*pi/180; % ���ת��radians, maximum steering angle (-MAXG < g < MAXG)
% RATEG= 20*pi/180; % �����ٶ�rad/s, maximum rate of change in steer angle
B = 1.5;%���ǳ���
global T_A2R
T_A2R=cell(4,1);
T_A2R{1} = [eye(3),[0.5;0.75;0];[0 0 0 1]];
T_A2R{2} = [eye(3),[-0.5;0.75;0];[0 0 0 1]];
T_A2R{3} = [eye(3),[0.5;-0.75;0];[0 0 0 1]];
T_A2R{4} = [eye(3),[-0.5;-0.75;0];[0 0 0 1]];
WHEELBASE= 2; % metres, vehicle wheel-base
DT_CONTROLS= 0.2; % ���Ƽ��ʱ��seconds, time interval between control signals

% % IMU��������
sigma_A = 2e-3; %���ٶȼƲ��������������
sigma_Az = 2e-3;
sigma_W = 0.02*pi/180; %�����ǲ������ٶ������������
sigma_IMU = [sigma_A^2;sigma_Az^2;sigma_W^2];

% ��������
global xl_R La
xl_R = [0.75;0.5;0]; %����������������ϵ�°�װλ��
La = 0.1; %����������ת��
THETA_RANGE = [0;120].*pi./180;
MAX_RANGE= 30.0; % metres
DT_OBSERVE= 4*DT_CONTROLS; % �������� seconds, time interval between observations

% �������
sigmaR= 1; % metres
sigmaB= (4.0*pi/180); % radians
sigmaTHETA = (4.0*pi/180);
R= [sigmaR^2 0 0; 0 sigmaB^2 0;0 0 sigmaTHETA^2];

% ���ݹ������� (Mahalanobis distances)
GATE_REJECT= 4.0; % maximum distance for association
GATE_AUGMENT= 25.0; % minimum distance for creation of new feature
% For 2-D observation:
%   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
%   - percent probability mass is: 1-sigma bounds 40%, 2-sigma 86%, 3-sigma 99%, 4-sigma 99.9%.

% waypoint proximity
AT_WAYPOINT= 1.0; % metres, distance from current waypoint at which to switch to next waypoint
NUMBER_LOOPS= 1; % number of loops through the waypoint list

% switches
SWITCH_CONTROL_NOISE= 1; % if 0, velocity and gamma are perfect
SWITCH_SENSOR_NOISE = 1; % if 0, measurements are perfect
SWITCH_INFLATE_NOISE= 0; % if 1, the estimated Q and R are inflated (ie, add stabilising noise)
SWITCH_HEADING_KNOWN= 0; % if 1, the vehicle heading is observed directly at each iteration
SWITCH_ASSOCIATION_KNOWN= 1; % if 1, associations are given, if 0, they are estimated using gates
SWITCH_BATCH_UPDATE= 1; % if 1, process scan in batch, if 0, process sequentially
SWITCH_SEED_RANDOM= 0; % if not 0, seed the randn() with its value at beginning of simulation (for repeatability)
SWITCH_USE_IEKF= 0; % if 1, use iterated EKF for updates, if 0, use normal EKF
SWITCH_PROFILE= 0; % if 1, turn on MatLab profiling to measure time consumed by simulator functions
SWITCH_GRAPHICS= 1; % if 0, avoids plotting most animation data to maximise simulation speed
