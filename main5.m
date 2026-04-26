%% main3的变型
clc,clear; close all; warning off;
%% 该代码用于跟踪参考点
load UCAVs_tasks3.mat;
load V_F_colors.mat;
%采样时间
delta_time = 1;
% 仿真迭代数目
num_simulation = 100;
% 无人机索引
% num_UCAV = 4; % 4个UCAV
max_graph_degrees = 2; % 每个图结点的最大邻结点个数
max_com_ranges = 3e4; % 通信范围为40km
max_detect_ranges = 250; % 雷达探测范围为250m
break_step = 20; % 每隔多长的次数画一次通信图
body_step = 3; % 每隔多长的次数画一次无人机body图
replanning_time = 10; % 哪个时刻开始重规划
replanning_flag = true; % 路径重规划标识符
% 速度系数
k_Vg = 1.5e-2;
mode = 'time';
sim_model = 'model2_diss.slx';
% 障碍物坐标
lon_move = - 0.009;
lat_move = 0.004;
hei_move = 20;
polygon_3D = [128.0650 + lon_move, 44.7700 + lat_move, 300 + hei_move;...
              128.0665 + lon_move, 44.7700 + lat_move + 0.001, 300 + hei_move;...
              128.0680 + lon_move, 44.7700 + lat_move, 300 + hei_move;...
              128.0680 + lon_move, 44.7680 + lat_move, 300 + hei_move;...
              128.0650 + lon_move, 44.7680 + lat_move, 300 + hei_move]; %障碍物坐标
goal_coordinates = [128.0598, 44.7722, 300];
% 初始化参数P
param; h = figure; 
hold on; grid on; box on;legend off; 
xlabel('Lon/X(m)');ylabel('Lat/Y(m)');zlabel('Hei/Z(m)');
mesh((LON - ucav_array(1).init_lng) * ucav_array(1).lon_scale, ...
     (LAT - ucav_array(1).init_lat) * ucav_array(1).lat_scale, ...
     DEM);
% 记录参数
command_array = cell(1,num_UCAV);
Va_c_array = cell(1,num_UCAV);
Va_array = cell(1,num_UCAV);
phi_c_array = cell(1,num_UCAV);
phi_array = cell(1,num_UCAV);
gamma_c_array = cell(1,num_UCAV);
gamma_array = cell(1,num_UCAV);
time_array = cell(1,num_UCAV);
theta_c_matrix = zeros(num_simulation,num_UCAV);
theta_matrix = zeros(num_simulation,num_UCAV);

pn_array = cell(1,num_UCAV);
pe_array = cell(1,num_UCAV);
pd_array = cell(1,num_UCAV);

pn_c_array = cell(1,num_UCAV);
pe_c_array = cell(1,num_UCAV);
pd_c_array = cell(1,num_UCAV);

ME_array = zeros(1,num_UCAV);
RMSE_array = zeros(1,num_UCAV);

%% 初始化ucav_array
for ucav_index = 1:num_UCAV
    % 初始化采样时间
    ucav_array(ucav_index).delta_t = delta_time;
    % 初始化P
    ucav_array(ucav_index).P = P;
    % 设置P
    [path_ref_x,path_ref_y,path_ref_z] = ucav_array(ucav_index).lng_lat_hei_to_xyz(ucav_array(ucav_index).path_ref(:,1), ...
                                                                          ucav_array(ucav_index).path_ref(:,2), ...
                                                                          ucav_array(ucav_index).path_ref(:,3));
    ucav_array(ucav_index).P.path_ref = [path_ref_x,path_ref_y,path_ref_z];
    % 获得waypoints
    ucav_array(ucav_index).P.waypoints = ucav_array(ucav_index).P.path_ref;
    % 配置坐标
    ucav_array(ucav_index).P.pn0 = path_ref_x(1); % x: north
    ucav_array(ucav_index).P.pe0 = path_ref_y(1); % y: east
    ucav_array(ucav_index).P.pd0 = -path_ref_z(1); % 这里是要修改的地方1
    ucav_array(ucav_index).P.u0 = 13;
    ucav_array(ucav_index).P.v0 = 0;
    ucav_array(ucav_index).P.w0 = 0;
    ucav_array(ucav_index).P.phi0 = 0;
    ucav_array(ucav_index).P.theta0 =  atan2(path_ref_z(2) - path_ref_z(1),...
                     norm([path_ref_y(2) - path_ref_y(1),...
                           path_ref_x(2) - path_ref_x(1)]));
    ucav_array(ucav_index).P.psi0 =  -atan2( path_ref_y(2) - path_ref_y(1),...
                     path_ref_x(2) - path_ref_x(1) ) - pi/2; % 这里是要修改的地方2
    ucav_array(ucav_index).P.p0 = 0;
    ucav_array(ucav_index).P.q0 = 0;
    ucav_array(ucav_index).P.r0 = 0;    
    ucav_array(ucav_index).P.pn_c = ucav_array(ucav_index).P.pn0; % x: north
    ucav_array(ucav_index).P.pe_c = ucav_array(ucav_index).P.pe0; % y: east
    ucav_array(ucav_index).P.pz_c = ucav_array(ucav_index).P.pd0; % 这里是要修改的地方1

    % 设置x_r, y_r, z_r, Vg_r, gamma_r, chi_r
    x_r = ucav_array(ucav_index).P.pe0; 
    y_r = ucav_array(ucav_index).P.pn0; 
    z_r = -ucav_array(ucav_index).P.pd0; 
    Vg_r = ucav_array(ucav_index).P.u0; 
    gamma_r = 0 ; 
    chi_r = 0;

    % 初始化ucav_array
    ucav_array(ucav_index).xr = x_r;
    ucav_array(ucav_index).yr = y_r;
    ucav_array(ucav_index).zr = z_r;
    ucav_array(ucav_index).Vg = Vg_r;
    
    % 画出当前路径图,画ref_waypoints
    h = draw_waypoints(ucav_array(ucav_index).P, h, ucav_index);
end

% 画障碍物图
height_min = 260;
[polygon_3D_x,polygon_3D_y,polygon_3D_z] = ucav_array(1).lng_lat_hei_to_xyz( ...
                                           polygon_3D(:,1), ...
                                           polygon_3D(:,2), ...
                                           polygon_3D(:,3) );

[goal_x,goal_y,goal_z] = ucav_array(1).lng_lat_hei_to_xyz( ...
                                            goal_coordinates(1),...
                                            goal_coordinates(2),...
                                            goal_coordinates(3));

polygon_3D_xyz = [polygon_3D_x , polygon_3D_y, polygon_3D_z];

polygon_3D_x_bar = mean(polygon_3D_x);
polygon_3D_y_bar = mean(polygon_3D_y);
polygon_3D_z_bar = mean(polygon_3D_z);

polygon_draw = [polygon_3D_xyz(:,1:3);
                polygon_3D_xyz(1,1:3)];
plot3(goal_x,goal_y,goal_z,'Marker','*','MarkerEdgeColor','g');
plot3(polygon_3D_x_bar,polygon_3D_y_bar,polygon_3D_z_bar,...
      'k-','Marker','+','MarkerEdgeColor','b','LineWidth',1.25);
num_points = 60;
plot3(polygon_3D_x_bar + max_detect_ranges * cos(linspace(0,2*pi,num_points)),...
      polygon_3D_y_bar + max_detect_ranges * sin(linspace(0,2*pi,num_points)),...
      polygon_3D_z_bar * ones(size(linspace(0,2*pi,num_points))), ...
      'm-.','Marker','none','LineWidth',1.25);

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

return;

%% 开始仿真
for k = 1:num_simulation
    clc; disp(['当前t:',num2str(k*delta_time),'s,已完成:',num2str(100 * k/num_simulation),'%']);
    %% 绘制UCAV的通信拓扑
    if ~mod(k,break_step)
         for i = 1:num_UCAV    
            if ~ucav_array(i).alive
                continue;
            end
            UCAV_index_array = [];
           % 声明距离矩阵
           distance_temps = zeros(1,num_UCAV);
           for j = 1:num_UCAV
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
           % 画ucav_i的相邻结点
           for l = UCAV_index_array
                for neighbor_id = ucav_array(l).neighbor_indexs 
                    start_point = [ucav_array(l).xr,...
                                   ucav_array(l).yr,...
                                   ucav_array(l).zr];
                    end_point = [ucav_array(neighbor_id).xr,...
                                   ucav_array(neighbor_id).yr,...
                                   ucav_array(neighbor_id).zr];
                    [parabola_x,parabola_y,parabola_z] = draw_3d_parabola(start_point, end_point, 340);
                    plot3(parabola_x,parabola_y,parabola_z,'Color','#0072BD','LineStyle','-.','MarkerSize',4,'LineWidth',0.75);
                end
           end
         end
    end

    %% UAV的当前状态更新，包括路径重规划
    for i = 1:num_UCAV
        if ~ucav_array(i).alive
            continue;
         end
        % 进行局部路径重规划,对ucav1
        if  (norm([ucav_array(i).xr - polygon_3D_x_bar,...
                  ucav_array(i).yr - polygon_3D_y_bar]) <= max_detect_ranges) && replanning_flag
            disp(['在t:',num2str(k * delta_time),'s时Replanning!']);
            % 重规划标识符
            replanning_flag = false; % 保证重规划一次
            % 选择当前的参考航路坐标为无人机的目标坐标
            ucav_array(i).ref_x_c = goal_coordinates(1);
            ucav_array(i).ref_y_c = goal_coordinates(2);
            ucav_array(i).ref_z_c = goal_coordinates(3);
            % 删除掉不合理的航路点
            ucav_array(i).delete_infeasible_ref_points(polygon_3D);
            % 插入重规划后的航路点
            tStart = tic;
            [lng_p,lat_p,hei_p] = replanning_Pc(ucav_array(i),...
                                               goal_coordinates, ...
                                               polygon_3D,h); 
            time_span = toc(tStart);
            % 画出新的重规划后的航路点
            [x_p,y_p,z_p] = ucav_array(i).lng_lat_hei_to_xyz(lng_p,lat_p,hei_p);
            plot3(x_p,y_p,z_p, 'Marker','x','Color','r','MarkerEdgeColor','b', ...
                  'LineStyle','-.','LineWidth',1,'DisplayName','Replanning-Point');
            % 将lng_p,lat_p,hei_p插入ref_path中
            ucav_array(i).insert_point(lng_p,lat_p,hei_p);
            % 更新ucav_array(i).P.waypoints
            [path_ref_x,path_ref_y,path_ref_z] = ucav_array(i).lng_lat_hei_to_xyz( ...
                                                  ucav_array(i).path_ref(:,1),...
                                                  ucav_array(i).path_ref(:,2),...
                                                  ucav_array(i).path_ref(:,3));
            
            ucav_array(i).P.waypoints = [path_ref_x,path_ref_y,path_ref_z];
        end
        
        %% UAV的动力学更新
        P = ucav_array(i).P;
        x_r = ucav_array(i).xr ;
        y_r = ucav_array(i).yr;
        z_r = ucav_array(i).zr;
        Vg_r = ucav_array(i).Vg;
        gamma_r = ucav_array(i).gamma;
        chi_r = ucav_array(i).chi;

        %% 获得参考点
        [P.pn_c,P.pe_c,P.pz_c] = get_track_point(P,x_r,y_r,z_r,Vg_r,gamma_r,chi_r);
        
        %% 获得参考速度
        if true
            theta = ucav_array(i).cal_theta(mode);
            % 计算当前的theta_c
            theta_c = ucav_array(i).aggr_neighbor_thetas(ucav_array,mode);
            % 计算其参考速度
            Va_c = ucav_array(i).guidance_Vg(theta_c,theta,k_Vg);
            % 计算对应的东西
            theta_c_matrix(k,i) = theta_c;
            theta_matrix(k,i) = theta;
        else
            Va_c = 13;
        end
        P.Va_c = Va_c;

        %% 每个Δt更新simulink
        %[P,out] = simulation2(P,delta_time);
        % 将数据P塞到workspace中
        assignin('base','P',P);
        % 仿真simulink,在[0,stopTime]时间段内仿真
        out = sim(sim_model, 'StopTime', num2str(delta_time));
        % 更新P值
        P.pn0 = out.pn.Data(end);
        P.pe0 = out.pe.Data(end);
        P.pd0 = -out.pd.Data(end);
        P.u0 = out.u.Data(end);
        P.v0 = out.v.Data(end);
        P.w0 = out.w.Data(end);
        P.phi0 = out.phi.Data(end);
        P.theta0 = out.theta.Data(end);
        P.psi0 = out.psi.Data(end);
        P.p0 = out.p.Data(end);
        P.q0 = out.q.Data(end);
        P.r0 = out.r.Data(end);
        
        %% 存储数据        
        Va_array{i} = [Va_array{i};out.Va.Data];
        Va_c_array{i} = [Va_c_array{i};out.Va_c.Data * ones(size(out.Va.Data))];
        phi_c_array{i} = [phi_c_array{i};out.phi_c.Data];
        phi_array{i} = [phi_array{i};out.phi.Data];
        gamma_c_array{i} = [gamma_c_array{i};out.gamma_c.Data];
        gamma_array{i} = [gamma_array{i};out.gamma.Data];
        command_array{i} = [command_array{i};out.command.Data];

        pn_array{i} = [pn_array{i};out.pn.Data];
        pe_array{i} = [pe_array{i};out.pe.Data];
        pd_array{i} = [pd_array{i};out.pd.Data];

        pn_c_array{i} = [pn_c_array{i};out.pn_c.Data];
        pe_c_array{i} = [pe_c_array{i};out.pe_c.Data];
        pd_c_array{i} = [pd_c_array{i};out.pz_c.Data];

        if ~isempty(time_array{i})
            current_time = time_array{i}(end);
        else
            current_time = 0;
        end
        time_array{i} = [time_array{i}; current_time + out.Va.Time];

        %% 在h上画图
        figure(h); axis equal;
        % 画出机体形状
        num_UAVs = 1;
        index_array = round(linspace(length(out.Va.Data)/8,length(out.Va.Data),num_UAVs));
        for ind_k = index_array
            pn = out.pn.Data(ind_k);
            pe = out.pe.Data(ind_k);
            pd = out.pd.Data(ind_k);
            phi = out.phi.Data(ind_k);
            theta = out.theta.Data(ind_k);
            psi = out.psi.Data(ind_k);
            % 开始画图
            aircraft_handle = drawBody(V,F,colors,...
                                   pe,pn,-pd,phi,theta,pi/2 - psi,...
                                   [], 'normal');
        end

        %% 获取UAV的当前state
        x_r = out.pn.Data(end); 
        y_r = out.pe.Data(end); 
        z_r = out.pd.Data(end); 
        Vg_r = out.Vg.Data(end); 
        gamma_r = out.theta.Data(end) - out.alpha.Data(end); 
        chi_r = out.chi.Data(end);

        %% 更新UAV的属性
        ucav_array(i).P = P;
        ucav_array(i).xr = x_r;
        ucav_array(i).yr = y_r;
        ucav_array(i).zr = z_r;
        ucav_array(i).Vg = Vg_r;
        ucav_array(i).gamma = gamma_r;
        ucav_array(i).chi = chi_r;
    end
end

ME_array = zeros(1,num_UCAV);
RMSE_array = zeros(1,num_UCAV);

for i = 1:num_UCAV

    waypoints = ucav_array(i).P.waypoints;

    ref_x_array = waypoints(:,1);
    ref_y_array = waypoints(:,2);
    ref_z_array = waypoints(:,3);

    [~,k] = min(abs(ref_x_array - pn_c_array{i}(end)));

    [ME,RMSE,MAXE,MINE] = get_path_error_indictors(pn_array{i},pe_array{i},pd_array{i}, ...
                                         ref_x_array(1:k),ref_y_array(1:k),ref_z_array(1:k));
    ME_array(i) = ME;
    RMSE_array(i) = RMSE;
end
num_thetas_matrix = 3;
theta_std_array = zeros(1,num_thetas_matrix);
theta_mean_array = zeros(1,num_thetas_matrix);
for l = 1:num_thetas_matrix
    theta_std_array(l) = std(theta_matrix(end-l+1,:));
    theta_mean_array(l) = mean(theta_matrix(end-l+1,:));
end
disp(['所有UCAVs的ME平均值：',num2str(mean(ME_array))]);
disp(['所有UCAVs的RMSE平均值：',num2str(mean(RMSE_array))]);
disp(['重规划时间:', num2str(time_span)]);
disp(['后三个时刻内\theta的std:',num2str(mean(theta_std_array)),...
                          ',mean:',num2str(mean(theta_mean_array))]);

return;
%% 画图显示
color_array = {'#A2142F','#0072BD','#77AC30',...
               '#7E2F8E','#EDB120','#D95319'};
% 先画指标曲线
figure;
hold on; grid on; box on;
xlabel('t (s)'); ylabel('\theta (s)'); legend();
t_sequences = ((1:num_simulation) * delta_time)';
for i = 1:num_UCAV
    plot(t_sequences,theta_matrix(:,i),'DisplayName',['\theta_',num2str(i)],'Linewidth',1.5)
end
% 再画参考跟踪曲线
figure;
% 画\gamma
hold on; grid on; box on;
xlabel('t (s)'); ylabel('\gamma (rad)'); legend();
for i = 1:num_UCAV
    plot(time_array{i},gamma_array{i}, 'Color',color_array{i},...
        'LineStyle','-','DisplayName',['\gamma_',num2str(i)],'Linewidth',1.5);
    plot(time_array{i},gamma_c_array{i}, 'Color',color_array{i},...
        'LineStyle','--','DisplayName',['\gamma_{c,',num2str(i),'}'],'Linewidth',1);
end
% 画\phi
figure;
hold on; grid on; box on;
xlabel('t (s)'); ylabel('\phi (rad)'); legend();
for i = 1:num_UCAV
    plot(time_array{i},phi_array{i}, 'Color',color_array{i},...
        'LineStyle','-','DisplayName',['\phi_',num2str(i)],'Linewidth',1.5);
    plot(time_array{i},phi_c_array{i}, 'Color',color_array{i},...
        'LineStyle','--','DisplayName',['\phi_{c,',num2str(i),'}'],'Linewidth',1);
end
% 画Va
figure;
hold on; grid on; box on;
xlabel('t (s)'); ylabel('Vg (m/s)'); legend();
for i = 1:num_UCAV
    plot(time_array{i},Va_array{i}, 'Color',color_array{i},...
        'LineStyle','-','DisplayName',['Vg_',num2str(i)],'Linewidth',1.5);
    plot(time_array{i},Va_c_array{i}, 'Color',color_array{i},...
        'LineStyle','--','DisplayName',['Vg_{c,',num2str(i),'}'],'Linewidth',1);
end
% 再画指令曲线
figure;
hold on; grid on; box on;
xlabel('t (s)'); ylabel('\delta_e (rad)'); legend();
for i = 1:num_UCAV
    plot(time_array{i},command_array{i}(:,3), 'Color',color_array{i},...
        'LineStyle','-','DisplayName',['\delta_{e',num2str(i),'}'],'Linewidth',1.5);
end

figure;
hold on; grid on; box on;
xlabel('t (s)'); ylabel('\delta_a (rad)'); legend();
for i = 1:num_UCAV
    plot(time_array{i},command_array{i}(:,4), 'Color',color_array{i},...
        'LineStyle','-','DisplayName',['\delta_{a',num2str(i),'}'],'Linewidth',1.5);
end

figure;
hold on; grid on; box on;
xlabel('t (s)'); ylabel('\delta_r (rad)'); legend();
for i = 1:num_UCAV
    plot(time_array{i},command_array{i}(:,5), 'Color',color_array{i},...
        'LineStyle','-','DisplayName',['\delta_{r',num2str(i),'}'],'Linewidth',1.5);
end

figure;
hold on; grid on; box on;
xlabel('t (s)'); ylabel('\delta_t (rad)'); legend();
for i = 1:num_UCAV
    plot(time_array{i},command_array{i}(:,6), 'Color',color_array{i},...
        'LineStyle','-','DisplayName',['\delta_{t',num2str(i),'}'],'Linewidth',1.5);
end