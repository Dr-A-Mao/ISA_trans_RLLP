% 李萨如曲线生成与可视化
% 支持二维和三维曲线，可自定义频率、振幅和相位参数

clear; clc; close all;

%% 参数设置
% North, East, Height的范围
North_range = 320; North_bais = 300;
East_range = 280; East_bais = 300;
Height_range = 50; Height_bais = 50;
color = [0.8, 0.3, 0.3];
numbers = 80;

% 基础参数
t = linspace(0, 20 * pi, numbers);  % 时间参数
dimension = 3;                 % 2=二维曲线，3=三维曲线

% 频率比 (决定曲线形状，有理数时闭合，无理数时非闭合)
f = [-0.1, -0.2, -0.2];                 % x, y, z方向频率

% 振幅设置
% x, y, z方向振幅
A = [North_range, East_range, Height_range];                 

% 相位偏移 (增加曲线复杂度)
phi = [pi/2, 0, pi/2];         % x, y, z方向相位

%% 生成李萨如曲线
x = A(1) * sin(f(1)*t + phi(1)) + North_bais;
y = A(2) * sin(f(2)*t + phi(2)) + East_bais;
z = A(3) * sin(f(3)*t + phi(3)) + Height_bais;

% 计算周长
diff_x = diff(x);
diff_y = diff(y);
diff_z = diff(z);

C = sum(sqrt((diff_x).^2 + (diff_y).^2 + (diff_z).^2));

%% 保存数据
path_ref = [x',y',-z'];
% 数据填充

path_ref = [path_ref(1,:);...
             (path_ref(1,:) * 0.5 + path_ref(2,:) * 0.5);...
             path_ref(2:end-1,:);...
             (path_ref(end-1,:) * 0.5 + path_ref(end,:) * 0.5);...
             path_ref(end,:)];

save('lissajous_CJA.mat','path_ref');

%% 画图
h = figure('Position', [100, 100, 800, 600]);hold on;
%ax = axes('Parent', gcf, 'Projection', 'perspective');
plot3(x, y, z, 'LineStyle', '--','LineWidth', 1, 'Color', color);

plot3(x(1), y(1), z(1), 'k*','MarkerSize',10);

scatter3(x, y, z, 8, t, 'filled', 'MarkerEdgeColor', 'none');
colorbar; colormap;
axis equal; grid on; box on;
xlabel('North (m)', 'FontSize', 12);
ylabel('East (m)', 'FontSize', 12);
zlabel('Height (m)', 'FontSize', 12);
view(3);
return;
%% CJA的pathfollowing算法
rho = 0.1;
L = diag([1 1 3]);
k = [0.005, 0.005, 0.005];
c1 = 3; c2 = 6;
tau = 1;
% 画速度场
Va = 13; 
d_hat = [5 5 5]';

f1 = @(t) A(1) * sin(f(1)*t + phi(1)) + North_bais;
f2 = @(t) A(2) * sin(f(2)*t + phi(2)) + East_bais;
f3 = @(t) A(3) * sin(f(3)*t + phi(3)) + Height_bais;

f1_dot = @(t) A(1) * f(1)* cos(f(1)*t + phi(1));
f2_dot = @(t) A(2) * f(2)* cos(f(2)*t + phi(2));
f3_dot = @(t) A(3) * f(3)* cos(f(3)*t + phi(3));

varphi1 = @(x,y,z,w)(x - f1(w * tau));
varphi2 = @(x,y,z,w)(y - f2(w * tau));
varphi3 = @(x,y,z,w)(z - f3(w * tau));

xi_d_dot = @(x,y,z,w)[ -rho^3 * tau * f1_dot(tau * w) - k(1) * rho^2 * varphi1(x,y,z,w);...
                       -rho^3 * tau * f2_dot(tau * w) - k(2) * rho^2 * varphi2(x,y,z,w);...
                       -rho^3 * tau * f3_dot(tau * w) - k(3) * rho^2 * varphi3(x,y,z,w);...
                       -rho^3 + rho^2 * tau * ( k(1) * varphi1(x,y,z,w) * f1_dot(tau * w) + ...
                                                k(2) * varphi2(x,y,z,w) * f2_dot(tau * w) + ...
                                                k(3) * varphi3(x,y,z,w) * f3_dot(tau * w)  )];


%% 可视化
figure(h);
for l = 1:length(x)        
    xi = x(l); yj = y(l); zk = z(l); wl = t(l);
    xi_d_dot_value = xi_d_dot(xi,yj,zk,wl);
    Vg_d_bar = xi_d_dot_value(1:3)/norm(xi_d_dot_value(1:3));
    P_d_dot_bar = Vg_d_bar;
    % 画出VF的向量线
    quiver3(xi,yj,zk,Vg_d_bar(1),Vg_d_bar(2),Vg_d_bar(3),20,'LineWidth',1.5,'Color','r'); % 风场扰动前的向量
    % 计算b,c
    b = P_d_dot_bar' * d_hat / Va;
    c = norm(d_hat) / Va;
    % 计算s_star
    s_star = b + sqrt(b^2 - c^2 + 1);
    % 计算v_1_d
    v_1_d = s_star * P_d_dot_bar - d_hat/Va; % v_1_d的norm为1
    % 画出v_1_d的向量线
    quiver3(xi,yj,zk,v_1_d(1),v_1_d(2),v_1_d(3),20,'LineWidth',1.5,'Color','b'); % 风场扰动后的向量
    % 风场扰动量
    quiver3(xi,yj,zk,d_hat(1)/norm(d_hat),d_hat(2)/norm(d_hat),d_hat(3)/norm(d_hat),20,'LineWidth',1.5,'Color','g'); % 风场扰动
    % 计算phi_d, theta_d
    theta_d = asin(v_1_d(3));
    % ----------无人机自己的关键信息----------
    psi = 0; % UAV的偏航角
    varepsilon = [cos(psi),sin(psi)]';
    % --------------------------------------
    % 计算psi_d_dot
    varepsilon_d = v_1_d(1:2)/norm(v_1_d(1:2));
    % 计算当前的psi_d
    psi_d = atan2(varepsilon_d(2),varepsilon_d(1));
    varepsilon_tilde = [-sin(psi_d),cos(psi_d)]';
    E = [0, 1; -1, 0];
    psi_d_dot =  varepsilon_d' * E * (-varepsilon)/(1 - varepsilon_d' * E * varepsilon_tilde ) ;
    phi_d = atan2( Va * psi_d_dot , 9.81);
end
 

