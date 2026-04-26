clear; clc; close all;

%% 参数设置
North_range = 600/2; North_bais = 300;
East_range = 600/2; East_bais = 300;
Height_range = 25; Height_bais = 50;
color = [0.8, 0.3, 0.3];
numbers = 70;

a = North_range;          % 椭圆长半轴
b = East_range;          % 椭圆短半轴
theta = atan(Height_range/North_range * 2);   % 绕x轴旋转角度
phi = 0;     % 绕y轴旋转角度
psi = 0;     % 绕z轴旋转角度
center = [North_bais, East_bais, Height_bais]; % 椭圆中心坐标

%% 生成椭圆上的点
t = linspace(0, 2*pi, numbers);  % 参数角度
x_2d = a * cos(t);           % 2D椭圆x坐标
y_2d = b * sin(t);           % 2D椭圆y坐标
z_2d = zeros(size(t));       % 2D椭圆z坐标(在xy平面)

%% 构造旋转矩阵
% 绕x轴旋转
Rx = [1, 0, 0;
      0, cos(theta), -sin(theta);
      0, sin(theta), cos(theta)];

% 绕y轴旋转
Ry = [cos(phi), 0, sin(phi);
      0, 1, 0;
      -sin(phi), 0, cos(phi)];

% 绕z轴旋转
Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0, 0, 1];

% 总旋转矩阵
R = Rz * Ry * Rx;

%% 将2D椭圆点转换为3D斜椭圆点
points_3d = zeros(3, length(t));
for i = 1:length(t)
    point_2d = [x_2d(i); y_2d(i); z_2d(i)];
    points_3d(:, i) = R * point_2d;  % 应用旋转变换
end

% 平移到指定中心
points_3d(1, :) = points_3d(1, :) + center(1);
points_3d(2, :) = points_3d(2, :) + center(2);
points_3d(3, :) = points_3d(3, :) + center(3);

%% 绘制椭圆
figure;
hold on;
grid on;
axis equal;
plot3(points_3d(1, :), points_3d(2, :), points_3d(3, :),...
      'color','r', 'LineWidth', 1,'LineStyle','--','Marker','x','MarkerEdgeColor','g');
plot3(center(1),center(2),center(3),'b+','LineWidth',1);
% % 绘制坐标轴以显示方向
% quiver3(center(1), center(2), center(3), ...
%         R(1,1), R(2,1), R(3,1), 'r', 'LineWidth', 1.5);  % 长轴方向
% quiver3(center(1), center(2), center(3), ...
%         R(1,2), R(2,2), R(3,2), 'g', 'LineWidth', 1.5);  % 短轴方向
% quiver3(center(1), center(2), center(3), ...
%         cross(R(:,1), R(:,2)), 'k', 'LineWidth', 1.5);   % 法向量方向

% 添加图例和标签
% legend('斜椭圆', '长轴方向', '短轴方向', '法向量方向', 'Location', 'Best');

xlabel('North (m)');
ylabel('East (m)');
zlabel('Height (m)');
%title('空间斜椭圆及其特征方向');
view(3);  % 设置3D视图

% 允许用户交互旋转查看
rotate3d on;
points_3d = points_3d';
path_ref = [points_3d(:,1),points_3d(:,2),-points_3d(:,3)]; 
save('ellipse.mat','path_ref');