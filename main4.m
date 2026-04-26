clc,clear; close all; warning off; % 采用QP算法优化
%% 该代码用于跟踪参考点
%采样时间
delta_time = 1;
% 仿真数目
num_simulation = 10;
%% 初始化参数P
param;
% 获得waypoints
P.waypoints = [P.path_ref(:,1:2),-P.path_ref(:,3)];
% 参考signals
P.Va_c = 13; P.gamma_c = pi/6; P.phi_c = 0;
x_r = 0; y_r = 0; z_r = 15; Vg_r = 13; gamma_r = pi/12; chi_r = 0;
gamma_c_range = [-pi/12, pi/6];
phi_c_range = [-pi/4, pi/4];
Va_c_range = [10, 18];
%% 画ref_waypoints
h = draw_waypoints(P); 
%% 开始仿真
for k = 1:num_simulation
    % 获得航路点和采样速度
    [P.pn_c,P.pe_c,P.pz_c] = get_track_point(P,x_r,y_r,z_r,Vg_r,gamma_r,chi_r);  
    %% 每个Δt更新simulink
    % 将数据P塞到workspace中
    assignin('base','P',P);
    % 仿真simulink,在[0,stopTime]时间段内仿真
    out = sim('model3.slx', 'StopTime', num2str(delta_time));
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
    % 画图显示
    h = draw_out(out,h);
    plot3(P.pn_c,P.pe_c,P.pz_c,'k.','LineWidth',1.5,'DisplayName','Target');
    % 获取UAV的当前state
    x_r = out.pn.Data(end); 
    y_r = out.pe.Data(end); 
    z_r = out.pd.Data(end); 
    Vg_r = out.Vg.Data(end); 
    Va_r = out.Va.Data(end);
    gamma_r = out.theta.Data(end) - out.alpha.Data(end); 
    chi_r = out.chi.Data(end);
    phi_r = out.phi.Data(end);
    % 计算
    chi_c = atan2( P.pe_c - y_r, P.pn_c - x_r );
    gamma_c = atan2( P.pz_c - z_r, norm([P.pn_c - x_r,P.pe_c - y_r]) );
    % 进行QP求解
    [gamma_c_dot,Va_c_dot,phi_c_dot] = QP_optimization(gamma_c - gamma_r,...
                                                       chi_c - chi_r, ...
                                                       P.Va_c - Va_r, ...
                                                       P.phi_c - phi_r, ...
                                                       P.Va_c, ...
                                                       P.phi_c);
    % 进行更新gamma_c, Va_c, phi_c
    P.gamma_c = P.gamma_c + delta_time * gamma_c_dot;
    if P.gamma_c >= gamma_c_range(2)
        P.gamma_c = gamma_c_range(2);
    elseif P.gamma_c <= gamma_c_range(1)
        P.gamma_c = gamma_c_range(1);
    end

    P.Va_c = P.Va_c + delta_time * Va_c_dot;
    if P.Va_c >= Va_c_range(2)
        P.Va_c = Va_c_range(2);
    elseif P.Va_c <= Va_c_range(1)
        P.Va_c = Va_c_range(1);
    end
    
    P.phi_c = P.phi_c + delta_time * phi_c_dot;
    if P.phi_c >= phi_c_range(2)
        P.phi_c = phi_c_range(2);
    elseif P.phi_c <= phi_c_range(1)
        P.phi_c = phi_c_range(1);
    end
    
    disp(['t=',num2str(k * delta_time),'s,Gamma_c:',num2str(P.gamma_c),', Va_c:',num2str(P.Va_c),', phi_c:',num2str(P.phi_c)]);
end

%% QP优化求解
function [gamma_c_dot,Va_c_dot,phi_c_dot] = QP_optimization(e_gamma,e_chi,e_Va,e_phi,Va_c,phi_c)
    % u_dot: [gamma_c_dot,Va_c_dot,phi_c_dot]'
    g = 9.81;
    % 计算误差向量
    e = [e_chi,e_gamma,e_Va,e_phi]';
    % 鲁棒阈值
    varepsilon_star = 1.5;
    % u_dot_max
    u_dot_max = [pi/4 * 5, 5 ,pi/6 * 5]';
    % u_dot_min
    u_dot_min = [-pi/4 * 5, -5,-pi/6 * 5]';
    % G
    G = [zeros(1,3);eye(3)];
    % R
    R = diag([0.1,0.5,0.1]);
    % 计算对应的b_gamma, b_Va, b_phi
    b_gamma = 0.1;
    b_Va = 0.5;
    b_phi = 0.1;
    % F(e,u)
    F = [ g/Va_c * tan(phi_c) - g/(Va_c - e_Va) * tan(phi_c - e_phi);...
              -b_gamma * e_gamma;...
              -b_Va * e_Va;...
              -b_phi * e_phi
            ];
    % QP优化
    A = [e' * G;eye(3);-eye(3)];
    b = [-e' * ( F + varepsilon_star/2 * e); u_dot_max; - u_dot_min];
    f = zeros(size(u_dot_max));
    % 求解得到，调用QP
    % 配置初值
    x0 = [0,0,0]';
    % 设置属性
    options = optimoptions('quadprog','Algorithm','active-set','MaxIterations',1500);
    % QP求解
    u_dot_optim = quadprog(R,f,A,b,[],[],[],[],x0,options);
    % 得到
    gamma_c_dot = u_dot_optim(1);
    Va_c_dot = u_dot_optim(2);
    phi_c_dot = u_dot_optim(3);

end
