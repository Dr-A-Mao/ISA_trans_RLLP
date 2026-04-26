clc,clear;close all;
load('UCAVs_tasks2.mat');
load('submap.mat');

for i = 1:length(ucav_array)
    ucav_array(i).init_lng = 127.99;
    ucav_array(i).init_lat = 44.6764;
end

%% 参数
k_scale = 0.25; % 画DEM图
scale = 3; % 路径稠密化参数

% Leader坐标
task_beginlng = 128.051; task_beginlat = 44.748;
task_endlng = 128.075; task_endlat = 44.765;

% Followers坐标
task_beginlng_array = [128.051, 128.0512, 128.0512, 128.0505];
task_beginlat_array = [44.748, 44.749, 44.747, 44.750];
task_beginhei_array = [1175.7, 1210, 1190, 1248];

num_UCAV = length(task_beginlng_array); % 无人机数目 

if  num_UCAV > length(ucav_array)
    num_iters = num_UCAV - length(ucav_array);
    for k = 1:num_iters
        ucav_array = [ucav_array,UCAV(127.99,44.6764)];
    end
end

%% 规划Leader路径
[path_temp,dist_temp,time_temp,cost_temp] = ucav_array(1).get_path_info( ...
                                                          task_beginlng,task_beginlat, ...
                                                          task_endlng,task_endlat);
% 进行scale缩放获得第一条reference waypoints
path_temp_1 = expand_path_points(path_temp(:,1),scale);
path_temp_2 = expand_path_points(path_temp(:,2),scale);
path_temp_3 = expand_path_points(path_temp(:,3),scale);
path_temp = [path_temp_1,path_temp_2,path_temp_3];
% 绘制第一条参考路径
ucav_array(1).path_ref = [path_temp(:,1:2), path_temp(:,3) * k_scale ];
ucav_array(1).current_lng = task_beginlng;
ucav_array(1).current_lat = task_beginlat;
[ucav_array(1).xr, ucav_array(1).yr, ucav_array(1).zr] = ucav_array(1).lng_lat_hei_to_xyz(task_beginlng,task_beginlat,path_temp(1,3) * k_scale );

%% 画图
figure; view([-128.05,-44.73,800]);
hold on; grid on; box on;
xlim([128.05, 128.1]); ylim([44.73, 44.78]); 
mesh(LON,LAT,DEM);
% 画轨迹图
% plot3(path_temp(1:end,1),path_temp(1:end,2),path_temp(1:end,3) * k_scale, 'rx--','LineWidth',1,'MarkerEdgeColor','r');

%% 规划剩余UCAVs的路径
color_array = {'r','#A2142F','#0072BD','k','#77AC30','#7E2F8E','#EDB120','#D95319'};

for k = 1:num_UCAV

    path_temp_1_k = path_temp_1 + task_beginlng_array(k) - task_beginlng;
    path_temp_2_k = path_temp_2 + task_beginlat_array(k) - task_beginlat;
    path_temp_3_k = path_temp_3 + task_beginhei_array(k) - ucav_array(1).zr/k_scale;
    
    ucav_array(k).path_ref = [path_temp_1_k, path_temp_2_k, path_temp_3_k * k_scale ];
    ucav_array(k).current_lng = task_beginlng_array(k);
    ucav_array(k).current_lat = task_beginlat_array(k);

    plot3(path_temp_1_k, path_temp_2_k, path_temp_3_k * k_scale, ...
             'color', color_array{k}, 'Marker','x', 'LineStyle','--',...
             'LineWidth',1,'MarkerEdgeColor',color_array{k});
end

%% 存储数据
save('UCAVs_tasks3.mat');


