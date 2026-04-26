clc,clear;close all;
load('UCAVs_tasks2.mat');
load('submap.mat');

for i = 1:length(ucav_array)
    ucav_array(i).init_lng = 127.99;
    ucav_array(i).init_lat = 44.6764;
end

%% config1

config_flag = 1;
if config_flag == 1
    task_beginlng = [128.051,128.053,128.098,128.099]; 
    task_beginlat = [44.748,44.7776,44.7765,44.747];
    task_endlng = 128.075 * ones(1,4); 
    task_endlat = 44.765  * ones(1,4);
elseif config_flag == 2
    task_beginlng = [128.098,128.099,128.098,128.097]; 
    task_beginlat = [44.7765,44.774,44.7755,44.773];
    task_endlng = [128.074, 128.076, 128.076, 128.077]; 
    task_endlat = [44.7650, 44.7658, 44.7642, 44.7650]; 
end

task_beginlng = [128.051,128.053,128.075,128.098,128.099,128.090,128.068]; 
task_beginlat = [44.748,44.7776,44.778,44.7765,44.747,44.74,44.75]; 

num_UCAV = length(task_beginlng);

if  num_UCAV > length(ucav_array)

    num_iters = num_UCAV - length(ucav_array);
    
    for k = 1:num_iters
        ucav_array = [ucav_array,UCAV(127.99,44.6764)];
    end
end



target_lng = zeros(size(task_beginlng)); 
target_lat = zeros(size(task_beginlat));
target_hei = zeros(size(task_beginlat));

k_scale = 0.25; % 画DEM图
scale = 3; % 路径稠密化参数

figure; 
hold on; grid on; box on;
mesh(LON,LAT,DEM);

for k = 1:num_UCAV
    % 声明起点和终点
    beginlng = task_beginlng(k);
    beginlat = task_beginlat(k);
    endlng = task_endlng(k);
    endlat = task_endlat(k);

    % 进行航路规划
    [path_temp,dist_temp,time_temp,cost_temp] = ucav_array(1).get_path_info( ...
                                                          beginlng,beginlat, ...
                                                          endlng,endlat);
    
    % 进行scale缩放
    path_temp_1 = expand_path_points(path_temp(:,1),scale);
    path_temp_2 = expand_path_points(path_temp(:,2),scale);
    path_temp_3 = expand_path_points(path_temp(:,3),scale);
    path_temp = [path_temp_1,path_temp_2,path_temp_3];

    target_lng(k) = path_temp(end-1,1);
    target_lat(k) = path_temp(end-1,2);
    target_hei(k) = path_temp(end-1,3)* k_scale;

    ucav_array(k).path_ref = [path_temp(:,1:2), path_temp(:,3) * k_scale ];
    ucav_array(k).current_lng = beginlng;
    ucav_array(k).current_lat = beginlat;

    [ucav_array(k).xr, ucav_array(k).yr, ucav_array(k).zr] = ucav_array(k).lng_lat_hei_to_xyz(beginlng,beginlat,path_temp(1,3) * k_scale );
    % 画轨迹图
    plot3(path_temp(1:end,1),path_temp(1:end,2),path_temp(1:end,3) * k_scale, ...
          'rx--','LineWidth',1,'MarkerEdgeColor','r');
end

% plot3([target_lng,target_lng(1)],[target_lat,target_lat(1)],[target_hei,target_hei(1)],'k-','Linewidth',1.5);

save('UCAVs_tasks3.mat');


