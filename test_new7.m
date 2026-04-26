%% 设计需要满足三个要求
% 1. 时间协同，要求同时达到目标\theta要接近；
% 2. 空间协同，要求无人机之间要避障不会互相碰撞；
% 3. 分布式通信，要求无人机之间有通信链路;
% 4. 鲁棒路径跟随，要求无人机飞行时能抵抗气流变化。
%% 导入.mat数据
% 请预先执行test2.m
clc,clear; close all; warning off;
load('UCAVs_tasks3.mat');
load('V_F_colors.mat');
%% 声明重要参数
k_scale = 1.5e-5; % UAV body的比例缩放系数
delta_t = 1; % 采样时间
tspan = 15; % 总时间 
% k_Vg = 1e-3; % 系数k_Vg
k_Vg = 1e-3; % 系数k_Vg
Vg_range = [9,13]; % 速度范围
num_iters = round(tspan/delta_t); % 总步数
UCAV_index_array = [1 2 3 4];
neighbors_array = {[2 3], [1], [1 4 2],[3 2]};
neighbor_betas_array = {[1e4 1e4],[1e4],[1e4 1e4 1e4],[1e4 1e4]};
flag = 'integrated';
coordinated_flag = false;
max_graph_degrees = 2; % 每个图结点的最大邻结点个数
max_com_ranges = 3e4; % 通信范围为40km
break_step = 30; % 每隔多长的次数画一次通信图
body_step = 3; % 每隔多长的次数画一次无人机body图
replanning_time = 500;
replanning_remain_delta_time = 50;
destroyed_times = [800];
destroyed_indexs = [1];
num_UCAVs = length(neighbors_array);
color_array = {'#A2142F','#0072BD','g',...
               '#7E2F8E','#EDB120','#D95319'};
polygon_3D = [128.0650, 44.7700 + 0.000, 300;...
              128.0665, 44.7700 + 0.003, 300;...
              128.0680, 44.7700 + 0.000, 300;...
              128.0680, 44.7680 + 0.000, 300;...
              128.0650, 44.7680 + 0.000, 300]; %障碍物坐标
goal_coordinates = [128.07, 44.7650, 280];
%% 配置参数并画图显示
param; % 初始化参数P
V = k_scale * V; % UAV body的比例缩放
h = figure;
%% 画ref_waypoints
hold on; grid on; box on;legend off;
xlabel('X/East/Lon');ylabel('Y/East/Lat');zlabel('Z/Height/Hei');
view([1000 300 5500]);

% mesh(LON,LAT,DEM); 
% % colormap('hsv');
% shading interp; 
% xlabel('Lon (°E)');
% ylabel('Lat (°W)');
% zlabel('Hei (m)');

for i = 1:num_UCAVs
    ucav_array(i).delta_t = delta_t; % 仿真时间步长0.5s
    ucav_array(i).zr = ucav_array(i).path_ref(1,3); % 初始高度(m)
    ucav_array(i).Vg_range = Vg_range; % 速度范围
    ucav_array(i).neighbor_indexs = neighbors_array{i};
    ucav_array(i).neighbor_betas = neighbor_betas_array{i};
    ucav_array(i).traj_length = 1e-6;
    ucav_array(i).cal_path_length();
    ucav_array(i).init_lng = 127.9999;
    ucav_array(i).init_lat = 44.6764;
    ucav_array(i).current_lng = ucav_array(i).path_ref(1,1);
    ucav_array(i).current_lat = ucav_array(i).path_ref(1,2);
    ucav_array(i).eta_lon = 0;
    ucav_array(i).eta_lat = 0;
    ucav_array(i).n_lf_range = [0,2.1];
    % 无人机的初始化参数P
    ucav_array(i).P = P;
    % 初始化高度
    [path_ref_x,path_ref_y,path_ref_z] = ucav_array(i).lng_lat_hei_to_xyz(ucav_array(i).path_ref(:,1), ...
                                                                          ucav_array(i).path_ref(:,2), ...
                                                                          ucav_array(i).path_ref(:,3));
    ucav_array(i).P.waypoints = [path_ref_x,path_ref_y,path_ref_z];
    
    ucav_array(i).P.pn0 = path_ref_y(1); % x: north, lat, y 
    ucav_array(i).P.pe0 = path_ref_x(1); % y: east, lon, x
    ucav_array(i).P.pd0 = -path_ref_z(1); % 这里是要修改的地方1
    
    ucav_array(i).P.u0 = 13;
    ucav_array(i).P.v0 = 0;
    ucav_array(i).P.w0 = 0;
    ucav_array(i).P.phi0 = 0;
    ucav_array(i).P.theta0 =  atan2(path_ref_z(2) - path_ref_z(1),...
                                     norm([path_ref_y(2) - path_ref_y(1),...
                                           path_ref_x(2) - path_ref_x(1)]));
    ucav_array(i).P.psi0 =  -atan2( path_ref_y(2) - path_ref_y(1),...
                                    path_ref_x(2) - path_ref_x(1) ); % 这里是要修改的地方2
    ucav_array(i).P.p0 = 0;
    ucav_array(i).P.q0 = 0;
    ucav_array(i).P.r0 = 0;
    % 声明ucav_array(i).out
    ucav_array(i).out.pn.Data = [ucav_array(i).yr];
    ucav_array(i).out.pe.Data = [ucav_array(i).xr];
    ucav_array(i).out.pd.Data = [ucav_array(i).zr];
    ucav_array(i).out.u.Data = [ucav_array(i).P.u0];
    ucav_array(i).out.v.Data = [ucav_array(i).P.v0];
    ucav_array(i).out.w.Data = [ucav_array(i).P.w0];
    ucav_array(i).out.phi.Data = [ucav_array(i).P.phi0];
    ucav_array(i).out.theta.Data = [ucav_array(i).P.theta0];
    ucav_array(i).out.psi.Data = [ucav_array(i).P.psi0];
    ucav_array(i).out.p.Data = [ucav_array(i).P.p0];
    ucav_array(i).out.q.Data = [ucav_array(i).P.q0];
    ucav_array(i).out.r.Data = [ucav_array(i).P.r0];
    % 画无人机的参考航路点
    plot3(ucav_array(i).path_ref(:,1),...
          ucav_array(i).path_ref(:,2),...
          ucav_array(i).path_ref(:,3),...
          'Color',color_array{i}, ...
          'LineStyle','--', 'LineWidth',0.8, 'Marker','x',...
          'MarkerEdgeColor',color_array{i},...
          'DisplayName',['无人机',num2str(i),'参考路径的航路点']);
end
% 画障碍物图
height_min = 260;
polygon_draw = [polygon_3D(:,1:3);
                polygon_3D(1,1:3)];
plot3(polygon_draw(:,1)',...
      polygon_draw(:,2)',...
      polygon_draw(:,3)', ...
     'k-','Marker','none','MarkerEdgeColor','r','LineWidth',1.5);
plot3(polygon_draw(:,1)',...
      polygon_draw(:,2)',...
      height_min * ones(size(polygon_draw(:,1)))', ...
      'k-','Marker','none','MarkerEdgeColor','r','LineWidth',1.5);
for i = 1:size(polygon_draw,1)
    plot3([polygon_draw(i,1),polygon_draw(i,1)],...
          [polygon_draw(i,2),polygon_draw(i,2)],...
          [height_min,polygon_draw(i,3)], ...
      'k-','Marker','none','MarkerEdgeColor','r','LineWidth',1.5);
end

%return;
%% 状态迭代更新
theta_matrix = zeros(num_iters,num_UCAVs);
theta_c_matrix = zeros(num_iters,num_UCAVs);
distance_matrix = zeros(num_iters,num_UCAVs);
Vg_matrix = zeros(num_iters,num_UCAVs);
mu_matrix = zeros(num_iters,num_UCAVs);
eta_matrix = zeros(num_iters,num_UCAVs);
chi_matrix = zeros(num_iters,num_UCAVs);
gamma_matrix = zeros(num_iters,num_UCAVs);
eta_lon_matrix = zeros(num_iters,num_UCAVs);
eta_lat_matrix = zeros(num_iters,num_UCAVs);
for k = 1:num_iters
    % 模拟外部损伤使得某些UCAVs毁损
    for l = 1:length(destroyed_indexs)
        if k * delta_t >= destroyed_times(l)
            ucav_array(destroyed_indexs(l)).alive = false;
        end
    end
    % 更新UCAVs的通信拓扑
    neighbors_array = {};
    neighbor_betas_array = {};
    UCAV_index_array = [];
    for i = 1:num_UCAVs    
        if ucav_array(i).alive
           % 声明距离矩阵
           distance_temps = zeros(1,num_UCAVs);
           for j = 1:num_UCAVs
              distance_temps(j) = norm([ucav_array(i).xr - ucav_array(j).xr,...
                                        ucav_array(i).yr - ucav_array(j).yr,...
                                        ucav_array(i).zr - ucav_array(j).zr]);
           end
           % 按距离降序排列
           [distance_sorts,sort_indexs] = sort(distance_temps);
           neighbor_indexs = [];
           beta_indexs = [];
           for j = 2:max_graph_degrees + 1
               UCAV_id = sort_indexs(j);
               if (distance_sorts(j) <= max_com_ranges) &&...
                                    ucav_array(UCAV_id).alive
                    neighbor_indexs = [neighbor_indexs,UCAV_id];
                    beta_indexs = [beta_indexs,1e4];
               end
           end
           UCAV_index_array = [UCAV_index_array,i];
           ucav_array(i).neighbor_indexs = neighbor_indexs;
           ucav_array(i).neighbor_betas = beta_indexs;
        end 
    end
    % 进行UCAVs的状态更新
     for i = 1:num_UCAVs
         if ~ucav_array(i).alive
            continue;
         end
        % 进行局部路径重规划,对ucav1
        if  (i == 1) && ...
            abs( k * delta_t - replanning_time) <= 0.1
            disp(['在t:',num2str(k * delta_t),'s时Replanning!']);
            % 删除掉不合理的航路点
            ucav_array(i).delete_infeasible_ref_points(polygon_3D);
            % 插入重规划后的航路点
            [lng_p,lat_p,hei_p] = replanning_Pc(ucav_array(i),...
                                               goal_coordinates, ...
                                               polygon_3D,h); 
            % 将lng_p,lat_p,hei_p插入ref_path中
            ucav_array(i).insert_point(lng_p,lat_p,hei_p);
        end
        % 确定参考路径点ref_x, ref_y, ref_z
        ref_lon = ucav_array(i).path_ref(:,1);
        ref_lat = ucav_array(i).path_ref(:,2);
        ref_hei = ucav_array(i).path_ref(:,3);
        [ref_x, ref_y, ref_z] = ucav_array(i).lng_lat_hei_to_xyz(ref_lon,ref_lat,ref_hei);
        
        % 确定UCAV_i的跟踪目标点x_p,y_p,z_p
        % [x_p,y_p,z_p] = ucav_array(i).get_track_point(ref_x,ref_y,ref_z);
        [x_p,y_p,z_p] = ucav_array(i).get_track_point2();

        % 转换成三维坐标
        [lng_p,lat_p,hei_p] = ucav_array(i).xyz_to_lng_lat_hei(x_p,y_p,z_p);

        disp(['--------',num2str(i),'-th UAV--------']);
        disp(['x_p,y_p,z_p:(',num2str(x_p),',',num2str(y_p),',',num2str(z_p),')']);
        disp(['x_r,y_r,z_r:(',num2str(ucav_array(i).xr),',',num2str(ucav_array(i).yr),',',num2str(ucav_array(i).zr),')']);

        % 画reference waypoints
        plot3(lng_p,lat_p,hei_p,'k.','LineStyle','none','LineWidth',1.5);
        
        % 计算参考速度
        theta = ucav_array(i).cal_theta(flag);
        if coordinated_flag
            % 计算当前的theta_c
            theta_c = ucav_array(i).aggr_neighbor_thetas(ucav_array,flag);
            % 计算其参考速度
            Vg_c = ucav_array(i).guidance_Vg(theta_c,theta,k_Vg);
            % 更新参考速度(需要被更新的)
            ucav_array(i).Vg = Vg_c;
            Va_c = Vg_c;
        else
            theta_c = theta;
            Va_c = Vg_range(1);
        end

        %% 开始simulink仿真
        out = ucav_array(i).dynamic_simulation(Va_c);

        %% 画出轨迹的机体图
        [pe_array,pn_array,pd_array] = ucav_array(i).xyz_to_lng_lat_hei( ...
                                                      out.pe.Data(2:end), ...
                                                      out.pn.Data(2:end), ...
                                                      out.pd.Data(2:end) );
        Va_array = out.Va.Data(2:end);
        phi_array = out.phi.Data(2:end);
        theta_array = out.theta.Data(2:end);
        psi_array = out.psi.Data(2:end);
        % 画图
        plot3(pe_array,pn_array,pd_array,'r.','LineWidth',0.5,'DisplayName','Trajectory');
        % 画出机体形状
        if ~mod(k,body_step)
            num_UAVs_markers = 1;
            index_array = round(linspace(length(Va_array)/8,length(Va_array),num_UAVs_markers));
            for ind = index_array
                pn = pn_array(ind);
                pe = pe_array(ind);
                pd = pd_array(ind);
                phi = phi_array(ind);
                theta = theta_array(ind);
                psi = psi_array(ind);
                % 开始画图
                aircraft_handle = drawBody(V,F,colors,...
                                       pn,pe,-pd,phi,theta,psi,...
                                       [], 'normal');
            end
        end
        
        % 保存数据
        eta_lon_matrix(k,i) = ucav_array(i).eta_lon;
        eta_lat_matrix(k,i) = ucav_array(i).eta_lat;
        
        theta_matrix(k,i) = theta;
        distance_matrix(k,i) = ucav_array(i).cal_theta('distance');
        theta_c_matrix(k,i) = theta_c;
        Vg_matrix(k,i) = ucav_array(i).Vg;
        mu_matrix(k,i) = 1 - ucav_array(i).traj_length / (ucav_array(i).refpath_length + 1e-5);
        eta_matrix(k,i) = distance_matrix(k,i)/ucav_array(i).max_distance;
     end
    % 画不同时刻的通信拓扑图
    if ~mod(k,break_step)
        draw_com_topology(ucav_array,UCAV_index_array);
        % break;
    end
    
    % 进度
    %clc;
    disp(['--------已运行',num2str(k/num_iters * 100),'%--------']);
    % 暂停
    pause(0.01);
end