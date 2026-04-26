% 用来画扰动噪声下的对比图
clc; clear; close all;
% 先画w0并计算
warning('off');
set(groot,'defaultLineLineWidth',1)
set(groot,'defaultAxesFontName','Times New Roman')
set(groot,'defaultAxesFontSize',10)
set(groot,'defaultAxesLabelFontSizeMultiplier',1) %标签字体大小的缩放因子，默认为1.1
set(groot,'defaultFigurePosition',[600 500 400 300]);
%% import data
load V_F_colors.mat;
load RLLP_lissajous_CJA_w0.mat;
time_state = state.Time;
data_state = state.Data;
time_command = command.Time;
data_command = command.Data;
time_compare = compare.Time;
data_compare = compare.Data;
time_attractor = attractor.Time;
data_attractor = attractor.Data;
%% get the main state sequences
NN = 0;
pn_array       = data_state(:,1+NN);       % inertial North position     
pe_array       = data_state(:,2+NN);       % inertial East position
pd_array       = data_state(:,3+NN);       % inertial Down position
u_array        = data_state(:,4+NN);       % body frame velocities
v_array        = data_state(:,5+NN);       
w_array        = data_state(:,6+NN);       
phi_array      = data_state(:,7+NN);       % roll angle         
theta_array    = data_state(:,8+NN);       % pitch angle     
psi_array      = data_state(:,9+NN);       % yaw angle     
p_array        = data_state(:,10+NN);      % roll rate
q_array        = data_state(:,11+NN);      % pitch rate     
r_array        = data_state(:,12+NN);      % yaw rate    
t_array        = data_state(:,13+NN);      % time

NN = NN + 13;
path     = data_state(:,1+NN:9+NN); 
NN = NN + 9;

uu = data_state(1,:);
% waypoints
num_waypoints = uu(1+NN);
waypoints     = reshape(uu(2+NN:5*num_waypoints+1+NN),5,num_waypoints)'; 
% reference pn,pe,pd
pn_ref = waypoints(:,1);
pe_ref = waypoints(:,2);
pd_ref = waypoints(:,3);
%% set the params
% plot size
S = 500; 
type_str = P.method;
% modify the map
buildings_n = [50,150,250,350,450,550];
buildings_e = [50,150,250,350,450,550];
heights = [ 20.8634, 5.4626, 59.0126, 16.5987,  3.7390, 35;...
            2.4827, 16.8860, 19.4869, 47.1025,  17.6579, 15;...
            45.1358, 45.0027, 12.0846, 47.8067, 5.0597, 45;...
            47.2394, 18.4623, 20.1956, 28.7604,  50.7702, 45;...
            24.5432,  5.5601,  4.8227,  2.9890,  2.1512, 45;...
            25, 35, 35, 35, 35, 5];
map.buildings_n = buildings_n;
map.buildings_e = buildings_e;
map.heights = heights;

%% draw the alpha,beta,delta_e,delta_a,delta_r,delta_t
alpha_array = data_command(:,1);
beta_array = data_command(:,2);
delta_e_array = data_command(:,3);
delta_a_array = data_command(:,4);
delta_r_array = data_command(:,5);
delta_t_array = data_command(:,6);
a_yc_array = data_command(:,7);
a_zc_array = data_command(:,8);
pn_c_array = data_command(:,9);
pe_c_array = data_command(:,10);
h_c_array = data_command(:,11);
%% 计算误差指标           
distance_array = sqrt((pe_c_array - pe_array).^2 +...
                      (pn_c_array - pn_array).^2 +...
                      (-h_c_array - pd_array).^2);
diff_pe_array = diff(pe_c_array);
nonzeros_time_index = find(diff_pe_array~=0);
errors_array = distance_array(nonzeros_time_index);
disp('--------下面显示误差指标--------');
ME = mean(errors_array); disp(['AE:',num2str(ME)]);
RMSE = std(errors_array); disp(['RMSE:',num2str(RMSE)]);
MAXE = max(errors_array); disp(['MAXE:',num2str(MAXE)]);
MINE = min(errors_array); disp(['MINE:',num2str(MINE)]);
disp('-------------------------------');
%% 画图
%% draw the trajectory
figure;
hold on;box on;grid on;
% drawing the waypoints of dubins trajectory
num_trajectory_points = length(pn_array);
% draw the trajectory
num_draw_points = 10;
trajectory_index_array = round(linspace(1,num_trajectory_points,...
                                        num_draw_points));
% draw the map
drawMap(map);
% plot the reference waypoints
plot3(waypoints(:,2),waypoints(:,1),-waypoints(:,3),...
      'LineWidth',1,'Color','k',...
      'LineStyle','--','Marker','x',...
      'MarkerEdgeColor','r',...
      'DisplayName','Reference waypoints');
% plot the actual trajectory
plot3(pe_array,pn_array,-pd_array,...
      'LineWidth',1.5,'Color','b',...
      'LineStyle','-.',...
      'DisplayName','Actual trajectory');
% draw the UAV body
for k = trajectory_index_array
    
    pn = pn_array(k);
    pe = pe_array(k);
    pd = pd_array(k);
    phi = phi_array(k);
    theta = theta_array(k);
    psi = psi_array(k);
  
    aircraft_handle = drawBody(V,F,colors,...
                           pn,pe,pd,phi,theta,psi,...
                           [], 'normal');
end
% draw the extra data
xlabel('East (m)');ylabel('North (m)'); zlabel('Height (m)');
x_min = min(waypoints(:,2)); x_max = max(waypoints(:,2));
y_min = min(waypoints(:,1)); y_max = max(waypoints(:,1));
z_min = min(waypoints(:,3)); z_max = max(waypoints(:,3));
axis([-map.width/5,map.width * 1.25,...
      -map.width/5,map.width * 1.25,...
      -20,100]);
% set the view angle for figure
view(-40,70); % legend();
% return;
%% 画剩下的图w1
hold on;
load RLLP_lissajous_CJA_w1.mat;
plot3(pe_array,pn_array,-pd_array,...
      'LineWidth',1.5,'Color','c',...
      'LineStyle','--',...
      'DisplayName','Actual trajectory');
% draw the UAV body
for k = trajectory_index_array
    
    pn = pn_array(k);
    pe = pe_array(k);
    pd = pd_array(k);
    phi = phi_array(k);
    theta = theta_array(k);
    psi = psi_array(k);
  
    aircraft_handle = drawBody(V,F,colors,...
                           pn,pe,pd,phi,theta,psi,...
                           [], 'normal');
end

%% 画剩下的图w2
hold on;
load RLLP_lissajous_CJA_w2.mat;
plot3(pe_array,pn_array,-pd_array,...
      'LineWidth',1.5,'Color','g',...
      'LineStyle','-.',...
      'DisplayName','Actual trajectory');
% draw the UAV body
for k = trajectory_index_array
    
    pn = pn_array(k);
    pe = pe_array(k);
    pd = pd_array(k);
    phi = phi_array(k);
    theta = theta_array(k);
    psi = psi_array(k);
  
    aircraft_handle = drawBody(V,F,colors,...
                           pn,pe,pd,phi,theta,psi,...
                           [], 'normal');
end

%% 画剩下的图w5
hold on;
load RLLP_lissajous_CJA_w5.mat;
plot3(pe_array,pn_array,-pd_array,...
      'LineWidth',1.5,'Color','y',...
      'LineStyle',':',...
      'DisplayName','Actual trajectory');
% draw the UAV body
for k = trajectory_index_array
    
    pn = pn_array(k);
    pe = pe_array(k);
    pd = pd_array(k);
    phi = phi_array(k);
    theta = theta_array(k);
    psi = psi_array(k);
  
    aircraft_handle = drawBody(V,F,colors,...
                           pn,pe,pd,phi,theta,psi,...
                           [], 'normal');
end

