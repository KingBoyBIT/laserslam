clear,clc,close all
global X Y Z nx ny nz r_w
configfile 
%建立待观测路标点
lm = build_map(X,Y,Z,300);

[data,dxz,XE_out,PE_out,da_table]= ekfslam_sim(lm,800);
result_plot