function [x_p,y_p,z_p] = get_track_point2(path_x,path_y,path_z,x_r,y_r,z_r,Vg_r,gamma_r,chi_r)
            % 路径参量
            path_eta_lat = atan2(path_y - y_r,path_x - x_r) - chi_r;
            path_eta_lon = atan2(path_z - z_r, sqrt((path_x - x_r).^2 + (path_y - y_r).^2)) - gamma_r;
            path_index =  find((abs(path_eta_lat) <= pi/2) & (abs(path_eta_lon) <= pi/2));
            if ~isempty(path_index)
                path_x = path_x(path_index);
                path_y = path_y(path_index);
                path_z = path_z(path_index);
            end
            % 搜索在路径path中最近投影点
            distance_r_path = sqrt((path_x-x_r).^2 + ...
                                   (path_y-y_r).^2 + ...
                                   (path_z-z_r).^2);
            [~,i] = min(distance_r_path); % 投影点索引为i
            next_distance_array = distance_r_path(i:end);
            % 设计期望的周期P_L和阻尼xi_L，q_L，计算重要参数
            xi_L = 0.707; P_L = 1;
            % 下面计算重要的制导率
            L = P_L * xi_L /pi * Vg_r;
            %% 需要修改为待跟踪的轨迹
            % 计算下一点的索引
            [~,k] = min(abs(next_distance_array - L));
            %待跟踪结点索引 
            p_index = i + k - 1;
            x_p = path_x(p_index);
            y_p = path_y(p_index);
            z_p = path_z(p_index);
end