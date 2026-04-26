function [x,y,z] = draw_3d_parabola(start_point, end_point, max_height)
    % 定义参数 t 的范围，生成 100 个均匀分布的点
    t = linspace(0, 1, 20);

    % 对于 x 和 y 方向，使用线性插值
    x = start_point(1) + (end_point(1) - start_point(1)) * t;
    y = start_point(2) + (end_point(2) - start_point(2)) * t;

    % 对于 z 方向，构建抛物线方程
    % 抛物线方程为 z = a*t^2 + b*t + c
    % 当 t = 0 时，z = start_point(3)
    % 当 t = 1 时，z = end_point(3)
    % 抛物线最高点在 t = 0.5 处，z = max_height

    c = start_point(3);
    % 根据 t=1 和 t=0.5 时的条件列方程求解 a 和 b
    A = [1, 1; 0.25, 0.5];
    b = [end_point(3) - c; max_height - c];
    coeffs = A \ b;
    a = coeffs(1);
    b = coeffs(2);

    % 计算 z 坐标
    z = a * t.^2 + b * t + c;

end    