function [Pc_lon,Pc_lat,Pc_hei,ref_Pc_x,ref_Pc_y,ref_Pc_z] = replanning_Pc(ucav,goal_coordinates,polygon_3D,h)
    if nargin == 3
        h = figure;
    end
    % goal_coordinates: 目标点三维坐标, [lon,lat,hei]
    % polygon_3D: 障碍物区域的三维坐标 N * 3, [lon,lat,hei]
    % rng(3);
    goal_lon_p =  goal_coordinates(1);
    goal_lat_p =  goal_coordinates(2);
    goal_hei_p =  goal_coordinates(3);

    [yp,xp,zp] = ucav.lng_lat_hei_to_xyz(goal_lon_p,goal_lat_p,goal_hei_p);

    obstacle.height = max(polygon_3D(:,3));
    obstacle.polygon = zeros(size(polygon_3D,1),2);

    for i = 1:size(polygon_3D,1) 
         [polygon_y,polygon_x,~] = ucav.lng_lat_hei_to_xyz(polygon_3D(i,1),...
                                                polygon_3D(i,2),...
                                                polygon_3D(i,3));
         obstacle.polygon(i,:) = [polygon_x,polygon_y];
    end
   
   % 超参数声明
   % rng(0);
   num = 1e4;
   min_d_xy = 15;
   max_iters = 20;
   Range_safe = [3,20]; % 安全高度差
   R_range = ucav.delta_t * ucav.Vg * 1 ; %预留三个采样步长的时间以用来避障 

   % 重心坐标
   weight_x = mean(obstacle.polygon(:,1));
   weight_y = mean(obstacle.polygon(:,2));

   %    R_min = max(sqrt((obstacle.polygon(:,1) - weight_x).^2 +...
   %                     (obstacle.polygon(:,2) - weight_y).^2));
    
   % 迭代产生新的waypoints
   ref_Pc_x = [];
   ref_Pc_y = [];
   ref_Pc_z = [];

   % 产生一个新的ucav_new用于处理数据
   ucav_new.delta_t = ucav.delta_t;
   ucav_new.xr = ucav.xr;
   ucav_new.yr = ucav.yr;
   ucav_new.zr = ucav.zr;
   ucav_new.Vg = ucav.Vg;
   ucav_new.chi = ucav.chi;
   ucav_new.gamma = ucav.gamma;
  
   for k = 1:max_iters
        [t_min,Pc_min] = get_optimal_Pc(ucav_new,obstacle,...
                                   [xp,yp,zp],num,R_range,Range_safe);
        % 存储数据
        ref_Pc_x = [ref_Pc_x;Pc_min(1)];
        ref_Pc_y = [ref_Pc_y;Pc_min(2)];
        ref_Pc_z = [ref_Pc_z;Pc_min(3)];
        disp(['第',num2str(k),'次重规划后的x_c:',num2str(Pc_min(1)),...
                                         ',y_c:',num2str(Pc_min(2)),...
                                         ',z_c:',num2str(Pc_min(3))]);
        % 进行ucav_new的状态更新
        ucav_new.chi = atan2(Pc_min(2) - ucav_new.yr,...
                             Pc_min(1) - ucav_new.xr);
        ucav_new.gamma = asin((Pc_min(3) - ucav_new.zr)/...
                             norm([Pc_min(2) - ucav_new.yr,...
                                   Pc_min(1) - ucav_new.xr]));
        ucav_new.xr = Pc_min(1);
        ucav_new.yr = Pc_min(2);
        ucav_new.zr = Pc_min(3);
        % 计算目标点和当前点距离
        d_xy = norm([ucav_new.xr - xp,...
                      ucav_new.yr - yp]);
        disp(['------Proceed for ',num2str(100 * k/max_iters),'%------']);
        if d_xy <= min_d_xy
            break;
        end
   end
   
   % Pc_min不应该是一个点，而应该是一系列航迹点

   [Pc_lat,Pc_lon,Pc_hei] = ucav.xyz_to_lng_lat_hei(ref_Pc_x,ref_Pc_y,ref_Pc_z);

   % 下面开始画图
   if false
       figure(h);
       % 画重规划后的航迹点
       plot3(Pc_lon,Pc_lat,Pc_hei,'gx','LineWidth',1,'DisplayName','Replanning-Point');
       
       polygon_draw = [polygon_3D(:,1:2);
                        polygon_3D(1,1:2)];
       figure(h); 
       hold on; box on; grid on; 
       xlabel('Lon (°E)'); ylabel('Lat (°W)'); zlabel('Hei (m)');
    
        plot3(polygon_draw(:,1)',...
          polygon_draw(:,2)',...
          obstacle.height * ones(size(polygon_draw(:,1)))', ...
          'k--','Marker','+','MarkerEdgeColor','r','LineWidth',1);
        plot3(polygon_draw(:,1)',...
              polygon_draw(:,2)',...
              0 * ones(size(polygon_draw(:,1)))', ...
              'k--','Marker','+','MarkerEdgeColor','r','LineWidth',1);
        for i = 1:length(polygon_draw(:,1))
            plot3([polygon_draw(i,1),polygon_draw(i,1)],...
                  [polygon_draw(i,2),polygon_draw(i,2)],...
                  [0,obstacle.height], ...
                  'k--','MarkerEdgeColor','r','LineWidth',1);
        end
        weight_y = mean(polygon_draw(:,1));
        weight_x = mean(polygon_draw(:,2));
        weight_z = mean(polygon_3D(:,3));
        plot3(weight_y,weight_x,weight_z,'ro');
        % 画重规划后的目标点
        plot3(Pc_lon,Pc_lat,Pc_hei,'gx','LineWidth',1,'DisplayName','Replanning-Point');
        % 画当前点
        plot3(ucav.current_lng,ucav.current_lat,ucav.current_hei,'bo','LineWidth',1,'DisplayName','Current-Point');
        % 画重规划前的目标点
        plot3(goal_lon_p,goal_lat_p,goal_hei_p,'gx','LineWidth',1,'DisplayName','Current-Point');
        % 画局部重规划的航迹
        plot3([ucav.current_lng;Pc_lon(1:end-1)],...
              [ucav.current_lat;Pc_lat(1:end-1)],...
              [ucav.current_hei;Pc_hei(1:end-1)],'b-.','LineWidth',1);
   end
end