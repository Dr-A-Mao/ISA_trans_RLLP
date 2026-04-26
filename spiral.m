% 绘制三维螺旋线
% 可通过修改参数调整螺旋线的形状、大小和方向

clear; clc; close all;
%% 螺旋线参数设置
North_range = 150; North_bais = 150;
East_range = -150; East_bais = 150;
Height_range = 20; Height_bais = 10;
color = [0.8, 0.3, 0.3];
num_circles = 4;
numbers = num_circles * 20;

t = linspace(0, 2 * pi * num_circles, numbers);  % 参数范围，控制螺旋的圈数
a = North_range;                                 % 螺旋半径
b = Height_range/num_circles;                    % 螺旋高度系数（每圈上升的高度）
c = 0;                                           % 螺旋旋转偏移量

%% 计算螺旋线坐标
% 基本螺旋线方程
% x = a * cos(t + c) + North_bais;
% y = a * sin(t + c) + East_bais;
% z = b * t + Height_bais;

x = 150 * cos(-1* t) + North_bais;
y = -150 * sin(-1* t) + East_bais;
z = 6 * t + Height_bais;

%% 绘制螺旋线
figure;
hold on;
grid on;
axis equal;
plot3(x, y, z, 'b', 'LineWidth', 1.5,'LineStyle','--',...
      'Marker','+','MarkerEdgeColor','r');

% 添加标签和标题
xlabel('North (m)');
ylabel('East (m)');
zlabel('Height (m)');

%% 保存数据
path_ref = [x',y',-z'];
save('spiral.mat','path_ref');
