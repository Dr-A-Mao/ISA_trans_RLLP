function [min_M,min_r,min_time] = get_M_r(waypoints,u,vg_u,s,vg_s,chi_s,...
                                          G_ur, P_ur_dB, G_m, P_m_dB, L, lambda_d)
    % 航路点    
    scale = 3; 

    % 给出对应的d(uc,ur)
    d_uc_ur = lambda_d/(4 * pi) * sqrt(G_m * G_ur/L) * 10^((P_ur_dB - P_m_dB)/20);

    waypoints_x = waypoints(:,1); x_s = s(1);
    waypoints_y = waypoints(:,2); y_s = s(2);
    waypoints_z = waypoints(:,3); z_s = s(3);

    waypoints_x = expand_path_points(waypoints_x,scale);
    waypoints_y = expand_path_points(waypoints_y,scale);
    waypoints_z = expand_path_points(waypoints_z,scale);

    [~,u_id] = min(sqrt((waypoints_x - u(1)).^2 + ...
                        (waypoints_y - u(2)).^2 + ...
                        (waypoints_z - u(3)).^2));

    L_u = 0; 
    min_cost = Inf; 
    min_M = [waypoints_x(1),waypoints_y(1),waypoints_z(1)];
    min_r = min_M;
    min_time = min_cost;
    
    num_waypoints = length(waypoints_x);
    for k = u_id:num_waypoints
        if k > 1
            L_u = L_u + norm([waypoints_x(k) - waypoints_x(k-1),...
                              waypoints_y(k) - waypoints_y(k-1),...
                              waypoints_z(k) - waypoints_z(k-1)]);
        end
        % 给出当前的x_r, y_r, z_r
        x_r = waypoints_x(k); 
        y_r = waypoints_y(k); 
        z_r = waypoints_z(k);
        % 给出当前的u_r
        u_r = [x_r,y_r,z_r];
        % 计算对应的d(s,ur)
        d_s_ur = norm([ x_s - x_r , y_s - y_r, z_s - z_r]);
        % 计算对应的(x_c,y_c,z_c)
        x_c = x_r + d_uc_ur/d_s_ur * (x_s - x_r);
        y_c = y_r + d_uc_ur/d_s_ur * (y_s - y_r);
        z_c = z_r + d_uc_ur/d_s_ur * (z_s - z_r);
        % 计算对应的d(s,uc)
        d_s_uc = d_s_ur - d_uc_ur;
        % 计算偏差
        theta_s = atan2(y_r - y_s,x_r - x_s) - chi_s;
        cost_f = abs( L_u/vg_u - d_s_uc/(vg_s * cos(theta_s)) );
        % 通过对比取最优的min_cost
        if cost_f < min_cost
            min_cost = cost_f;
            min_M = u_r;
            min_r = [ x_c, y_c, z_c];
            min_time = L_u/vg_u;
        end
    end
    
end