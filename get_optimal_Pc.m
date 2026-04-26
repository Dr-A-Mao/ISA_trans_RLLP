function [t_min,Pc_min] = get_optimal_Pc(ucav,obstacle,X2,num,R_range,Range_safe)
    if nargin < 4
        num = 1000;
        R_range = 500; % 横向扩张距离
        Range_safe = [-20,15]; % 安全飞行高度的增量
    end
    % ucav的状态量: xr,yr,zr,Vg,chi,gamma

    polygon = obstacle.polygon;
    X1 = [ucav.xr,ucav.yr,ucav.zr];
    % 计算凸多边形的重心
    weight_x = mean(polygon(:,1));
    weight_y = mean(polygon(:,2));
    distances = sqrt((polygon(:,1) - weight_x).^2 +...
                 (polygon(:,2) - weight_y).^2);
    R_max = max(distances);
    R_min = min(distances);
    t_min = inf;
    Pc_min = X2;
    % 产生num个从[R_min, R_max + R_range]的环形区域内的数
    for i = 1:num
        R = R_max + R_range * rand;
        theta = -pi + 2 * pi * rand;
        z = ucav.zr + Range_safe(1) +...
             (Range_safe(2) - Range_safe(1)) * rand;

        point_x = weight_x + R * cos(theta);
        point_y = weight_y + R * sin(theta);
        point_z = z;

        Pc = [point_x,point_y,point_z];

        num_u = 25;
        InConvexPolygon_flag = false;
        for l = linspace(0,1,num_u)
            Pc_X2 = l * Pc + (1-l) * X2; 
            X1_Pc = l * X1 + (1-l) * Pc;
            if InConvexPolygon(Pc_X2,obstacle) ||...
                    InConvexPolygon(X1_Pc,obstacle)
                InConvexPolygon_flag = true;
                break;
            end
        end
        if ~InConvexPolygon_flag
            % 计算时间
            t = get_replanning_time(ucav,X2,Pc);
            if t < t_min
                t_min = t;
                Pc_min = Pc;
            end
        end
    end
end