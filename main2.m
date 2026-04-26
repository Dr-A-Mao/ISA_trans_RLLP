%% 用于进行Duffingmodel的仿真
clc,clear;close all;rng(0);
%% 配置Matlab环境参数
% set(groot,'defaultLineLineWidth',1);
% set(groot,'defaultAxesFontName','Times New Roman');
% set(groot,'defaultAxesFontSize',10);
% set(groot,'defaultAxesLabelFontSizeMultiplier',1);
% set(groot,'defaultFigurePosition',[600 500 400 300]);
%% 算法参数
load('P.mat');
%% 纵向MIMO-PI控制器
disp('--------开始计算纵向MIMO-PI控制器--------');
rng_seed_lon = 0; % 随机数种子
optimization_type_lon = 'fminsearch'; % 采用的优化器算法
alg_type = 2; % EVP function 
A_lon = P.A_lon(1:end-1,1:end-1);
B_lon = P.B_lon(1:end-1,:);
[K_P_lon,K_I_lon] = solve_K_opt(A_lon,B_lon, ...
                                rng_seed_lon,optimization_type_lon,alg_type);
[RK_lon,IK_lon] = get_RK_IK(A_lon,B_lon,K_P_lon,K_I_lon);
K_lon = [K_P_lon,K_I_lon];

% %% 修改数据小扰动
% epsilon_K_P = 0;
% K_P_lon_new = K_P_lon - epsilon_K_P * [eye(2),eye(2)];
% 
% epsilon_K_I = 0;
% K_I_lon_new = K_I_lon - epsilon_K_I * [eye(2),eye(2)];
% 
% [RK_lon,IK_lon] = get_RK_IK(A_lon,B_lon,K_P_lon_new,K_I_lon_new);
% 
% save('Lon.mat','K_P_lon','K_I_lon','RK_lon','IK_lon','K_lon');
% return;
%% 横向MIMO-PI控制器
disp('--------开始计算横向MIMO-PI控制器--------');
optimization_type_lat = 'fminsearch'; % 采用的优化器算法
alg_type = 2; % EVP function 
rng_seed_lat = 5; % 随机数种子
A_lat = P.A_lat(1:end-1,1:end-1);
B_lat = P.B_lat(1:end-1,:);
[K_P_lat,K_I_lat] = solve_K_opt(A_lat,B_lat,...
                                rng_seed_lat,optimization_type_lat,alg_type);
[RK_lat,IK_lat] = get_RK_IK(A_lat,B_lat,K_P_lat,K_I_lat);
K_lat = [K_P_lat,K_I_lat];
% save('Lat.mat','K_P_lat','K_I_lat','RK_lat','IK_lat','K_lat');

