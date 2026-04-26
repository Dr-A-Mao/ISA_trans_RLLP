clc,clear; close all; warning off;
%% 该代码用于跟踪参考点
load UCAVs_tasks3.mat;
load V_F_colors.mat;
%采样时间
delta_time = 1.5;
% 仿真数目
num_simulation = 25;
% 无人机索引
ucav_index = 1;
%% 初始化参数P
param; 
% 设置P, x: north, lat, pn ;
%        y: east, lon, pe
[path_ref_x,path_ref_y,path_ref_z] = ucav_array(ucav_index).lng_lat_hei_to_xyz(ucav_array(ucav_index).path_ref(:,1), ...
                                                                      ucav_array(ucav_index).path_ref(:,2), ...
                                                                      ucav_array(ucav_index).path_ref(:,3));
P.path_ref = [path_ref_x,path_ref_y,path_ref_z]; % path_ref_x: lon, east, Pe ; path_ref_y: lat, north, pn
% 获得waypoints
P.waypoints = P.path_ref; 
% 配置坐标
P.pe0 = path_ref_x(1); % y: east
P.pn0 = path_ref_y(1); % x: north
P.pd0 = -path_ref_z(1); % 这里是要修改的地方1

P.u0 = 13;
P.v0 = 0;
P.w0 = 0;
P.phi0 = 0;
P.theta0 =  atan2(path_ref_z(2) - path_ref_z(1),...
                 norm([path_ref_y(2) - path_ref_y(1),...
                       path_ref_x(2) - path_ref_x(1)]));
P.psi0 =  atan2( path_ref_x(2) - path_ref_x(1),...
                 path_ref_y(2) - path_ref_y(1) ) ; % 这里是要修改的地方2
P.p0 = 0;
P.q0 = 0;
P.r0 = 0;
% 设置x_r (pn), y_r (pe), z_r (pd)
x_r = P.pn0; 
y_r = P.pe0; 
z_r = -P.pd0; 
Vg_r = P.u0; 
gamma_r = 0 ; 
chi_r = P.psi0; % 计算chi_r，修改之处
% 输出初始时刻的状态
disp(['初始时刻UAV状态:x_r:',num2str(x_r),...
      ',y_r:',num2str(y_r),...
      ',z_r:',num2str(z_r),...
      ',Vg_r:',num2str(Vg_r),...
      ',gamma_r:',num2str(gamma_r),...
      ',chi_r:',num2str(chi_r)]);
%% 画ref_waypoints
h = figure;
h = draw_waypoints(P,h,1);
% 在当前h基础上再画一个平行的参考航路点
start_x = 8000; 
start_y = 6800; 
start_z = 294; 
P_new = P;
P_new.waypoints = P.waypoints + [start_y - P.waypoints(1,1),...
                                 start_x - P.waypoints(1,2),...
                                 start_z- P.waypoints(1,3)];
h = draw_waypoints(P_new,h,2);
return;
%% 开始仿真
for k = 1:num_simulation
    %% UAV的当前状态
    %% 获得采样点和采样速度
    [P.pn_c,P.pe_c,P.pz_c] = get_track_point(P,x_r,y_r,z_r,Vg_r,gamma_r,chi_r); % x_r: north, y_r: east
    % disp(['P.pn_c:',num2str(P.pn_c),',P.pe_c:',num2str(P.pe_c),',P.pz_c:',num2str(P.pz_c)]);
    P.Va_c = 13;
    %% 不知道代码为什么出了问题2026.3.8，暂时用不到希望后面有时间回来再改改吧 ！！！！！
    %% 每个Δt更新simulink
    %[P,out] = simulation2(P,delta_time);
    % 将数据P塞到workspace中
    assignin('base','P',P);
    % 仿真simulink,在[0,stopTime]时间段内仿真
    out = sim('model2_1.slx', 'StopTime', num2str(delta_time));
    % 更新P值
    P.pn0 = out.pn.Data(end);
    P.pe0 = out.pe.Data(end);
    P.pd0 = -out.pd.Data(end);
    P.u0 = out.u.Data(end);
    P.v0 = out.v.Data(end);
    P.w0 = out.w.Data(end);
    P.phi0 = out.phi.Data(end);
    P.theta0 = out.theta.Data(end);
    P.psi0 = out.psi.Data(end);
    P.p0 = out.p.Data(end);
    P.q0 = out.q.Data(end);
    P.r0 = out.r.Data(end);
    %h = draw_out(out,h);
    figure(h);
    hold on; grid on; box on;legend off; axis equal;
    plot3(P.pn_c,P.pe_c,P.pz_c,'b+');
    xlabel('North,Lat,Pn(m)');ylabel('East,Lon,Pe(m)');zlabel('Down(m)');
    % 画出机体形状
    num_UAVs = 1;
    index_array = round(linspace(length(out.Va.Data)/8,length(out.Va.Data),num_UAVs));
    for k = index_array
        pn = out.pn.Data(k);
        pe = out.pe.Data(k);
        pd = out.pd.Data(k);
        phi = out.phi.Data(k);
        theta = out.theta.Data(k);
        psi = out.psi.Data(k);
        % 开始画图
        aircraft_handle = drawBody(V,F,colors,...
                                   pe,pn,-pd,phi,theta,pi/2 - psi,...
                                   [], 'normal');
    end
    %% 获取UAV的当前state
    x_r = out.pn.Data(end); 
    y_r = out.pe.Data(end); 
    z_r = out.pd.Data(end); 
    Vg_r = out.Vg.Data(end); 
    gamma_r = out.theta.Data(end) - out.alpha.Data(end); 
    chi_r = out.chi.Data(end);
    %disp(['UAV在第',num2str(k),'时刻的chi_r:',num2str(chi_r)]);
end