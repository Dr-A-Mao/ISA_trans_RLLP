%% main function is used to draw the picture
% 数据对比图在这里画
clc;%close all;
warning('off');
set(groot,'defaultLineLineWidth',1)
set(groot,'defaultAxesFontName','Times New Roman')
set(groot,'defaultAxesFontSize',10)
set(groot,'defaultAxesLabelFontSizeMultiplier',1) %标签字体大小的缩放因子，默认为1.1
set(groot,'defaultFigurePosition',[600 500 400 300]);
%% import data
load V_F_colors.mat;
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
      'LineStyle','-',...
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

%return;
%% draw the attractor sequences
figure;
hold on;box on;
plot(time_attractor,data_attractor, ...
    'LineWidth',1.5,'Color','b');
xlabel('t(s)'); ylabel('I(f)');

figure;
hold on; box on;
plot(time_command,alpha_array * 180/pi,'b-','LineWidth',1.5,'DisplayName','\alpha');
xlabel('t (s)'); ylabel('\alpha (deg)');legend();

figure;
hold on; box on;
plot(time_command,beta_array * 180/pi,'b-','LineWidth',1.5,'DisplayName','\beta');
xlabel('t (s)'); ylabel('\beta (deg)');legend();

figure;
hold on; box on;
plot(time_command,delta_e_array,'b-','LineWidth',1.5,'DisplayName','\delta_e');
xlabel('t (s)'); ylabel('\delta_e (rad)');legend();

figure;
hold on; box on;
plot(time_command,delta_a_array,'b-','LineWidth',1.5,'DisplayName','\delta_a');
xlabel('t (s)'); ylabel('\delta_a (rad)');legend();

figure;
hold on; box on;
plot(time_command,delta_r_array,'b-','LineWidth',1.5,'DisplayName','\delta_r');
xlabel('t (s)'); ylabel('\delta_r (rad)');legend();

figure;
hold on; box on;
plot(time_command,delta_t_array,'b-','LineWidth',1.5,'DisplayName','\delta_t');
xlabel('t (s)'); ylabel('\delta_t (rad)');legend();

figure;
hold on; box on;
plot(time_command,a_yc_array,'b-','LineWidth',1.5,'DisplayName','a_{yc}');
xlabel('t (s)'); ylabel('a_{yc} (m/s^2)');legend();

figure;
hold on; box on; 
a_zc_array(find(a_zc_array  >= 9.8)) = 9.8;
plot(time_command,a_zc_array,'b-','LineWidth',1.5,'DisplayName','a_{zc}');
xlabel('t (s)'); ylabel('a_{zc} (m/s^2)');legend();

figure;
hold on; box on;
plot(time_command,pn_c_array,'r--.','LineWidth',1.5,'DisplayName','Ref north');
plot(time_state,pn_array,'b-','LineWidth',1.5,'DisplayName','Actual north');
xlabel('t (s)'); ylabel('North (m)');legend();

figure;
hold on; box on;
plot(time_command,pe_c_array,'r--.','LineWidth',1.5,'DisplayName','Ref east');
plot(time_state,pe_array,'b-','LineWidth',1.5,'DisplayName','Actual east');
xlabel('t (s)'); ylabel('East (m)');legend();

figure;
hold on; box on;
plot(time_command,h_c_array,'r--.','LineWidth',1.5,'DisplayName','Ref height');
plot(time_state,-pd_array,'b-','LineWidth',1.5,'DisplayName','Actual height');
xlabel('t (s)'); ylabel('Height (m)');legend();

%% draw eta^{lon}, eta^{lat}, d
figure;
hold on; box on;

plot(time_state,distance_array,'b-','LineWidth',1.5,'DisplayName','Distance');
plot(time_state,zeros(size(time_state)),'r--','LineWidth',1.5,'DisplayName','Baseline of origin');
xlabel('t (s)'); 
ylabel('Distance from the current points to reference points (m)');legend();


gamma_array = theta_array - alpha_array;
chi_array = psi_array - beta_array;

chi_c_array = atan2(pe_c_array - pe_array, pn_c_array - pn_array);
gamma_c_array = atan2(-h_c_array - pd_array,...
                      sqrt((pe_c_array - pe_array).^2 + (pn_c_array - pn_array).^2) );

eta_lon_array = gamma_c_array - gamma_array;
eta_lat_array = chi_c_array - chi_array;

figure;
hold on; box on;
plot(time_state,eta_lon_array,'b-','LineWidth',1.5,'DisplayName','\eta^{lon}');
plot(time_state,zeros(size(time_state)),'r--','LineWidth',1.5,'DisplayName','origin');
xlabel('t (s)');  ylabel('\eta^{lon} (rad)');legend();

figure;
hold on; box on;
plot(time_state,eta_lat_array,'b-','LineWidth',1.5,'DisplayName','\eta^{lat}');
plot(time_state,zeros(size(time_state)),'r--','LineWidth',1.5,'DisplayName','origin');
xlabel('t (s)');  ylabel('\eta^{lat} (rad)');legend();

figure;
hold on; box on;
plot(time_state,sqrt(eta_lat_array.^2 + eta_lon_array.^2),'b-','LineWidth',1.5,'DisplayName','||\eta||_2');
plot(time_state,zeros(size(time_state)),'r--','LineWidth',1.5,'DisplayName','origin');
xlabel('t (s)');  ylabel('||\eta||_2 (rad)');legend();

%% 计算T(e0)和上界的关系!!!
figure;hold on; box on; grid on; 
num_after = 5;L_c = 1.2;
T_e0_array = diff(t_array(nonzeros_time_index + num_after));
Dis_array = distance_array(nonzeros_time_index + num_after);
Vg_array = sqrt(u_array(nonzeros_time_index + num_after).^2 + ...
                v_array(nonzeros_time_index + num_after).^2 + ...
                w_array(nonzeros_time_index + num_after).^2);
eta_lon_key_array = eta_lon_array(nonzeros_time_index + num_after);
eta_lat_key_array = eta_lat_array(nonzeros_time_index + num_after);
% 假设L_c极为接近0
T_e0_hat_array = Dis_array./(Vg_array .* cos(eta_lon_key_array) .* cos(eta_lat_key_array) - L_c);
plot(t_array(nonzeros_time_index),[T_e0_array(1);T_e0_array], ...
     'b-x','LineWidth',1.5,'DisplayName','T(e_0)','MarkerEdgeColor','auto');
plot(t_array(nonzeros_time_index),T_e0_hat_array, ...
     'r--+','LineWidth',1.5,'DisplayName','Upperbound of T(e_0)','MarkerEdgeColor','auto');
xlabel('t(s)');ylabel('Comparsion of T(e_0) and its upperbound');legend();

% 为了显示的更清晰易懂
figure;hold on; box on; grid on; 
x_title = cell(1,length(T_e0_hat_array));
for k = 1:length(T_e0_hat_array)
    x_title{k} = [num2str(t_array(nonzeros_time_index(k)),'%.2f'),'s'];
end
vals_title = [[T_e0_array(1);T_e0_array]';T_e0_hat_array'];
b = bar(categorical(x_title),vals_title);
legend('T(e_s)','Estimated upperbound of T(e_s)');
xlabel('t_s(s)');ylabel('Comparsion of T(e_s) and its upperbound');legend();

return;
%% draw the compare datas
Va_c_array = data_compare(:,1);
Va_array = data_compare(:,2);
phi_c_array = data_compare(:,3);
phi_array = data_compare(:,4);
n_lf_c_array = data_compare(:,5);
n_lf_array = data_compare(:,6);

figure;
hold on; box on;
plot(time_compare,Va_c_array,'r--','LineWidth',1.5,'DisplayName','V_{gc}');
plot(time_compare,Va_array,'b-','LineWidth',1.5,'DisplayName','V_g');
xlabel('t (s)'); ylabel('V_g (m/s)');legend();

figure;
hold on; box on;
plot(time_compare,phi_c_array * 180/pi,'r--','LineWidth',1.5,'DisplayName','\phi_c');
plot(time_compare,phi_array * 180/pi,'b-','LineWidth',1.5,'DisplayName','\phi');
xlabel('t (s)'); ylabel('\phi (deg)');legend();

figure;
hold on; box on;
plot(time_compare,n_lf_c_array,'r--','LineWidth',1.5,'DisplayName','n_{lf,c}');
plot(time_compare,n_lf_array,'b-','LineWidth',1.5,'DisplayName','n_{lf}');
xlabel('t (s)'); ylabel('n_{lf}');legend();

