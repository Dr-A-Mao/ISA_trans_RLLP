clc;clear; close all;

IJRNC_data_file_path = 'IJRNC_data_fig9.csv';
TIT_data_file_path = 'TIT_data_fig12b.csv';

IJRNC_data = csvread(IJRNC_data_file_path);
TIT_data = csvread(TIT_data_file_path);

IJRNC_x = IJRNC_data(:,1);
IJRNC_y = IJRNC_data(:,2);

TIT_x = TIT_data(:,1);
TIT_y = TIT_data(:,2);

%% 画IJRNC的对比图
theta = linspace(0,2*pi,40); 
R = 125;
circle_x = R * cos(theta);
circle_y = R * sin(theta);

distance_errors = abs(sqrt((IJRNC_x).^2 + (IJRNC_y).^2) - R);

disp('--------下面显示误差指标--------');
ME = mean(distance_errors); disp(['AE:',num2str(ME)]);
RMSE = std(distance_errors); disp(['RMSE:',num2str(RMSE)]);
MAXE = max(distance_errors); disp(['MAXE:',num2str(MAXE)]);
MINE = min(distance_errors); disp(['MINE:',num2str(MINE)]);
disp('-------------------------------');

figure ; hold on; box on; axis equal;
xlabel('x(m)'); ylabel('y(m)');
plot(IJRNC_x,IJRNC_y,'color','b','Displayname','IJRNC','Linestyle','none','Marker','.');
plot(circle_x,circle_y,'r--','LineWidth',1.5);

% 数据导出
path_ref = [circle_x' + 125, circle_y' + 125, zeros(size(circle_x))'];
save('circle.mat','path_ref');

%% 画TIT的对比图
t = linspace(0,2,100);
Bezier_x = 20 *  t.^2 + 50 * t - 25;
Bezier_y = -65 * t.^2 + 50 * t - 10;

figure ; hold on; box on; axis equal;
xlabel('x(m)'); ylabel('y(m)');
plot(TIT_x,TIT_y,'color','b','Displayname','TIV','Linestyle','none','Marker','.');
plot(Bezier_x,Bezier_y,'r--','LineWidth',1.5);
