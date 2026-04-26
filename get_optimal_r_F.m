%% 2026.4.6工作
function [optim_r,optim_F,optim_cost] = get_optimal_r_F(U_array,S_array, G_m, P_m_dB, L, lambda_d)
    % 每个U_array还要包括G_ur, P_ur_dB
    % 每个S_array包含和U_array相同的属性
    optim_cost = inf;
    optim_F = [U_array(1).xr,U_array(1).yr,U_array(1).zr];
    optim_r = [U_array(1).xr,U_array(1).yr,U_array(1).zr];
    %% 从S_array中的每个UAV开始遍历
    for uav_S = S_array
        S = [uav_S.xr, uav_S.yr, uav_S.zr];
        S_vg = uav_S.Vg;
        S_chi = uav_S.chi;
        % 下面开始遍历所有的U_array
        for uav_U = U_array
            U = [uav_U.xr, uav_U.yr, uav_S.zr];
            U_vg = uav_U.Vg;
            U_waypoints = [uav_U.P.waypoints(:,2),...
                           uav_U.P.waypoints(:,1),...
                           uav_U.P.waypoints(:,3)];
            U_G =uav_U.G_ur;
            U_P_dB = uav_U.P_ur_dB;
            % 计算对应的时间
            [min_M,min_r,min_time] = get_M_r(U_waypoints,U,U_vg,S,S_vg,S_chi,...
                                             U_G, U_P_dB, G_m, P_m_dB, L, lambda_d);
            % 进行寻优计算
            if min_time <= optim_cost
                optim_cost = min_time;
                optim_F = S;
                optim_r = min_r;
            end
        end
    end
    
end