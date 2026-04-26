function out = path_follow_RLLP_optimal(in,P)
  %% 根据dubins路径给出的参考状态量
  NN = 0;
  flag        = in(1+NN);
  Va_d        = in(2+NN);
  c_path      = [in(3+NN); in(4+NN); in(5+NN)];
  psi_path    = in(6+NN);
  gam_path    = in(7+NN);
  rho_path    = in(8+NN);
  lambda_path = in(9+NN);
  %% UAV的真实16-DOF状态量，有用的不多
  NN = NN + 9;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
  alpha     = in(5+NN);
  beta      = in(6+NN);
  phi       = in(7+NN);
  theta     = in(8+NN);
  chi       = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
   r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  % bx      = in(17+NN);
  % by      = in(18+NN);
  % bz      = in(19+NN);
  NN = NN + 19;
  t         = in(1+NN);

  gamma_dot = in(2+NN);

  gamma = theta - alpha;
  
  % command airspeed equal to desired airspeed
  Va_c = Va_d;    

  persistent path_data % store path data to detect when it changes
  persistent spiral_index
  persistent varphi_old
  persistent psiTilde
  % 目标点坐标
  persistent pn_c
  persistent pe_c
  persistent h_c
  % 最优系数
  persistent k_opt

  % logic to ensure smooth switching to new spiral and straight line paths
  if t<P.Ts,
      path_data = [flag; Va_d; c_path; psi_path; gam_path; rho_path; lambda_path];
      spiral_index = 0;
      varphi_old = 0;
      psiTilde   = 0;

      pn_c = 0;
      pe_c = 0;
      h_c = 0;
  else
      path_data_new = [flag; Va_d; c_path; psi_path; gam_path; rho_path; lambda_path];
      if norm(path_data-path_data_new)>.1,  % if the path has changed, then reset the spiral angle
          path_data = path_data_new;
          spiral_index = 0;
          varphi_old = 0;
      end 
  end
  
  % alpha1:  longitudinal plane
  n_lon = [-sin(psi_path); cos(psi_path); 0];

  % alpha2:  lateral plane
  q = [cos(psi_path)*cos(gam_path); sin(psi_path)*cos(gam_path); -sin(gam_path)];
  n_lat = cross(q, n_lon);
  alpha2 = n_lat'*([pn;pe;-h]-c_path);
 
  % obtain n_lf
  n_lf = obtain_n_lf(gamma_dot,Va,gamma,phi,P);
  
  % 关键的对比组对于跟踪点方式
  [pn_c_new,pe_c_new,h_c_new] = select_ref_point(pn,pe,h,...
                                  Va,chi,gamma,...
                                  P.path_ref(:,1),...
                                  P.path_ref(:,2),...
                                  -P.path_ref(:,3));
  if norm([pn_c_new - pn_c, pe_c_new - pe_c, h_c_new - h_c]) >= 1  % 若目标点发生改变 
      target_change = true;
      pn_c = pn_c_new;
      pe_c = pe_c_new;
      h_c = h_c_new;
  else
      target_change = false;
  end


  % 计算关键变量
  chi_c = atan2( pe_c - pe, pn_c - pn);
  gamma_c = atan2( h_c - h, norm([ pe_c - pe , pn_c - pn ]));
  
  % 计算eta_lon, eta_lat
  eta_lon = gamma_c - gamma;
  eta_lat = chi_c - chi;

  eta = [eta_lat,eta_lon]';

  % convex optimization
  gravity = 9.8;

  a_min = [-gravity/sqrt(2); -2.1 * gravity];
  a_max = [ gravity/sqrt(2);  2.1 * gravity];

  F = [eta_lat, 0, eta_lon, 0;...
       0, eta_lat, 0, eta_lon];
  
  R = diag([0.1,0.1]);

  G = [1,0,0,-1;...
       0,1,1,0;...
       1,0,0,1];

  if Va == 0
      Va = P.Va_c;
  end

  Lambda = diag([-Va * cos(phi)/cos(beta), -Va]);

  b = [0; gravity * cos(gamma)];

  optimization_type = P.optimization_type; 
  % 若目标点改变则执行一次凸优化
  if target_change 
      if strcmp(optimization_type,'energy')
         k_opt = optimization_energy(F,G,Lambda,R,b,gravity,P);
      elseif strcmp(optimization_type,'robustness') ||...
             strcmp(optimization_type,'attractor')
         k_opt = optimization_robustness(F,Lambda,b,P);
      elseif strcmp(optimization_type,'analytical')
          [k_opt1,~] = optimization_analytical3(Lambda,b,a_min,a_max,eta);
          k_opt = k_opt1;
          % [k_opt2,R_min2] = optimization_analytical2(Lambda,b,a_min,a_max,eta);
          % k_opt = k_opt2;
      end
  end

  % 计算I(f)
  K = [k_opt(1:2),k_opt(3:4)];  
  K_norm = norm(K);
  K_inverse_norm = norm(inv(K));
  R_f = -max(eig(K + K'));
  I_f =  K_norm * K_inverse_norm/R_f;
  % 计算当前加速度
  a_opt = b + Lambda * F * k_opt;
  a_yc = a_opt(1); a_zc = a_opt(2);

  % obtain n_lf_c and phi_c
  [n_lf_c,phi_c] = obtain_phi_n_c(a_yc,a_zc,P);
  if true
      % obtain gamma_c
      gamma_c = obtain_gamma_c(n_lf_c,n_lf,t,P); %% 在这里进行修改！！！
  end
  % create output
  out = [Va_c; gamma_c; phi_c; I_f; n_lf_c; n_lf;a_yc;a_zc;pn_c;pe_c;h_c];
end

%-----------------------------------------------------------
% analytical solution optimization
function [k_opt,R_min] = optimization_analytical0(Lambda,b,a_min,a_max,eta, ...
                                                  R_star,delta_theta,delta_s)
    if nargin == 5
        R_star = 0.1;
        delta_theta = 0.1;
        delta_s = 0.1;
    end
    beta_min = 5/180 * pi;
    c_beta_min = cos(beta_min);
    theta_max = asin(2*sqrt(c_beta_min)/(c_beta_min + 1 ));

    M = 100;
    Q1_star = @(theta)[cos(theta),-sin(theta);...
                       sin(theta),cos(theta)];
    R_min = inf; theta_star = 0; s_star = - delta_s;

    for theta = -theta_max:delta_theta:theta_max
        Q_star = Q1_star(theta);
        for s = -delta_s: -delta_s: -delta_s * M 
            Q1_eta = Q_star * eta;
            Q1_eta_left = inv(Lambda) * (a_min - b)/s ;
            Q1_eta_right = inv(Lambda) * (a_max - b)/s;
            Q1_eta_left_ = min(Q1_eta_left,Q1_eta_right);
            Q1_eta_right_ = max(Q1_eta_left,Q1_eta_right);
            flag = all(Q1_eta_left_<=  Q1_eta)&&all(Q1_eta<= Q1_eta_right_);
            if ~flag
                break;
            end

            % 计算
            R = -1/s * 1/(1-cos(theta)) *...
                       1/((1+cos(theta))/(1-cos(theta)) - c_beta_min) *...
                       1/c_beta_min;

            if R < R_min
                R_min = R;
                theta_star = theta;
                s_star = s;
            end
        end
    end
    K_opt = s_star * Q1_star(theta_star);
    disp('---K_opt---:');
    disp(['   |',num2str(K_opt(1,1)),',',num2str(K_opt(1,2)),'|   ']);
    disp(['   |',num2str(K_opt(2,1)),',',num2str(K_opt(2,2)),'|   ']);
    k_opt = K_opt(:);
end



%-----------------------------------------------------------
% analytical solution optimization
function [k_opt,R_min] = optimization_analytical1(Lambda,b,a_min,a_max,eta, ...
                                                  R_star,delta_theta,delta_s)
    if nargin == 5
        R_star = 0.1;
        delta_theta = 0.1;
        delta_s = 0.1;
    end
    delta_s_max = 3;
    Q1_star = @(theta)[cos(theta),-sin(theta);...
                       sin(theta),cos(theta)];
    R_min = -R_star; theta_star = -pi/2; s_star = - R_star/cos(theta_star);
    for theta = -pi/2:delta_theta:pi/2
        Q_star = Q1_star(theta);
        for s = -R_star/cos(theta): -delta_s: -delta_s_max
            Q1_eta = Q_star * eta;
            Q1_eta_left = inv(Lambda) * (a_min - b)/s ;
            Q1_eta_right = inv(Lambda) * (a_max - b)/s;
            Q1_eta_left_ = min(Q1_eta_left,Q1_eta_right);
            Q1_eta_right_ = max(Q1_eta_left,Q1_eta_right);
            flag = all(Q1_eta_left_<=  Q1_eta)&&all(Q1_eta<= Q1_eta_right_);
            if ~flag
                break;
            end
            if s * cos(theta) < R_min
                R_min = s * cos(theta);
                theta_star = theta;
                s_star = s;
            end
        end
    end
    K_opt = s_star * Q1_star(theta_star);
    disp('---K_opt---:');
    disp(['   |',num2str(K_opt(1,1)),',',num2str(K_opt(1,2)),'|   ']);
    disp(['   |',num2str(K_opt(2,1)),',',num2str(K_opt(2,2)),'|   ']);
    k_opt = K_opt(:);
end

%-----------------------------------------------------------
% analytical solution optimization
function [k_opt,R_min] = optimization_analytical2(Lambda,b,a_min,a_max,eta, ...
                                                  R_star,delta_theta,delta_s)
    if nargin == 5
        R_star = 0.1;
        delta_theta = 0.1;
        delta_s = 0.1;
    end
    delta_s_max = 3;
    Q2_star = @(theta)[cos(theta),sin(theta);...
                       sin(theta),-cos(theta)];
    R_min = -R_star; theta_star = 0; s_star = - R_star/2;
    for theta = 0:delta_theta:pi*2
        Q_star = Q2_star(theta);
        for s = -R_star/2: -delta_s: -delta_s_max
            Q2_eta = Q_star * eta;
            Q2_eta_left = inv(Lambda) * (a_min - b)/s ;
            Q2_eta_right = inv(Lambda) * (a_max - b)/s;
            Q2_eta_left_ = min(Q2_eta_left,Q2_eta_right);
            Q2_eta_right_ = max(Q2_eta_left,Q2_eta_right);
            flag = all(Q2_eta_left_<=  Q2_eta)&&all(Q2_eta<= Q2_eta_right_);
            if ~flag
                break;
            end
            if s < R_min/2
                R_min = s * 2;
                theta_star = theta;
                s_star = s;
            end
        end
    end
    K_opt = s_star * Q2_star(theta_star);
    k_opt = K_opt(:);
end

%-----------------------------------------------------------
% analytical solution optimization
function [k_opt,R_min] = optimization_analytical3(Lambda,b,a_min,a_max,eta, ...
                                                  R_star,delta_theta,delta_s)
    if nargin == 5
        R_star = 0.1;
        delta_theta = 0.1;
        delta_s = 0.001;
    end
    
    M = 1e3;

    R_min = inf; s_star = 0;

    for s = -delta_s: -delta_s: -M 
        flag1 = all((a_min -b)<= Lambda * eta * s ) &&...
                all(Lambda * eta * s <= (a_max -b));
        if ~flag1
            s_star = s;
            break;
        end
    end
    
    K_opt = s_star * eye(2);
    disp('---K_opt---:');
    disp(['   |',num2str(K_opt(1,1)),',',num2str(K_opt(1,2)),'|   ']);
    disp(['   |',num2str(K_opt(2,1)),',',num2str(K_opt(2,2)),'|   ']);
    k_opt = K_opt(:);
end

%-----------------------------------------------------------
% fminsearch optimization of robustness
function k_opt = optimization_robustness(F,Lambda,b,P)
  gravity = 9.8;

  a_min = [-gravity/sqrt(2); -2.1 * gravity];
  a_max = [ gravity/sqrt(2);  2.1 * gravity];
  
  k0 = [-P.k_chi,0,0,-P.k_gamma]';
  % 挑选fun
  if strcmp(P.optimization_type,'robustness')
        fun = @(k)robustness_cost(k,Lambda,F,a_min,a_max,b);
  elseif strcmp(P.optimization_type,'attractor')
        fun = @(k)attractor_cost(k,Lambda,F,a_min,a_max,b);
  end

  [k_opt,fval] = fminsearch(fun,k0,...
                            optimset('MaxFunEvals', 250 * 4,...
                                     'MaxIter', 250 * 4));
  if fval == inf
     k_opt = [-P.k_chi,0,0,-P.k_gamma]';
  end

  a_opt = b + Lambda * F * k_opt;

  energy = 0.5 * a_opt' * diag([0.1,0.1]) * a_opt;
  
  disp(['找到最优解k_opt=[',num2str(k_opt'),...
                  '],energy=',num2str(energy)]);
end

function cost_value = robustness_cost(k,Lambda,F,a_min,a_max,b)
    K = [k(1:2),k(3:4)];  
    R_f = -max(eig(K + K'));
    if all( Lambda * F * k <= (a_max - b) ) &&...
       all( Lambda * F * k >= (a_min - b) ) &&...
       R_f > 0
       cost_value = 1/R_f;
    else
       cost_value = inf;
    end
end

function cost_value = attractor_cost(k,Lambda,F,a_min,a_max,b)
    K = [k(1:2),k(3:4)];  

    K_norm = norm(K);
    K_inverse_norm = norm(inv(K));
    R_f = -max(eig(K + K'));

    if all( Lambda * F * k <= (a_max - b) ) &&...
       all( Lambda * F * k >= (a_min - b) ) &&...
       R_f > 0 &&...
       K_inverse_norm < inf   
       cost_value = K_norm * K_inverse_norm/R_f;
    else
       cost_value = inf;
    end
end

%-----------------------------------------------------------
% convex optimization of acceleration energy
function k_opt = optimization_energy(F,G,Lambda,R,b,gravity,P)
  % key parameters
  R_star = 1; 
  
  varepsilon1 = 0.5;
  varepsilon2 = 0.5;
  Infi = 10000;

  a_min = [-gravity/sqrt(2); -2.1 * gravity/sqrt(2)];
  a_max = [ gravity/sqrt(2);  2.1 * gravity/sqrt(2)];

  g1 = [-varepsilon1;-varepsilon2;-Infi];
  g2 = [varepsilon1;varepsilon2;...
        -sqrt(varepsilon1^2 + varepsilon2^2) - R_star];
  
  H = F' * Lambda' * R * Lambda * F;
  f = F' * Lambda' * R' * b;
  
  A_leq = [Lambda * F; G;...
          -Lambda * F; -G];
  b_leq = [a_max - b; g2;...
           -(a_min - b); -g1];

  [k_opt,~,exitflag] = quadprog(H,f,A_leq,b_leq);
  
  if exitflag ~= 1
     k_opt = [-P.k_chi,0,0,-P.k_gamma]';
  end
  % disp(['exitflag=',num2str(exitflag)]);
end

%-----------------------------------------------------------
% select reference points
function [x_p,y_p,z_p] = select_ref_point(x_r,y_r,z_r,Vg_r,chi_r,gamma_r,...
                                          path_x,path_y,path_z)
    % 路径参量
    path_eta_lat = atan2(path_y - y_r,path_x - x_r) - chi_r;
    path_eta_lon = atan2(path_z - z_r, sqrt((path_x - x_r).^2 + (path_y - y_r).^2)) - gamma_r;
    path_index =  find((abs(path_eta_lat) <= pi/2) & (abs(path_eta_lon) <= pi/2));
    path_x = path_x(path_index);
    path_y = path_y(path_index);
    path_z = path_z(path_index);
    % 搜索在路径path中最近投影点
    distance_r_path = sqrt((path_x-x_r).^2 + ...
                           (path_y-y_r).^2 + ...
                           (path_z-z_r).^2);
    [~,i] = min(distance_r_path); % 投影点索引为i
    next_distance_array = distance_r_path(i:end);
    % 设计期望的周期P_L和阻尼xi_L，q_L，计算重要参数
    xi_L = 0.707; P_L = 10;
    % 下面计算重要的制导率
    q_L = P_L * xi_L /pi;
    L = q_L * Vg_r;
    %% 需要修改为待跟踪的轨迹
    % 计算下一点的索引
    [~,k] = min(abs(next_distance_array - L));
    %待跟踪结点索引 
    p_index = i + k - 1;
    x_p = path_x(p_index);
    y_p = path_y(p_index);
    z_p = path_z(p_index);
end

%-----------------------------------------------------------
% obtain accelerations
function [a_yc,a_zc] = guidance_law(Vg,phi,chi,gamma,...
                                    chi_c,gamma_c,...
                                    chi_c_dot,gamma_c_dot,...
                                    P,type)
    g = P.gravity;
    
    eta_lat = chi_c - chi;
    eta_lon = gamma_c - gamma;
    
    %% 函数类别
    [f_chi,f_gamma] = f(eta_lat,eta_lon,chi_c_dot,gamma_c_dot,P,type);

    a_yc = - Vg * cos(phi) * f_chi;
    a_zc = - Vg * f_gamma + g * cos(gamma);
end

% function handles for gamma and chi
function [f_chi,f_gamma] = f(e_chi,e_gamma,chi_c_dot,gamma_c_dot,P,type)
    
    k_chi = P.k_chi;
    k_gamma = P.k_gamma;

    % 不同的相关性函数
    % 1 : 线性函数; 2: 正弦函数; 3: tan函数; 4: 其他函数;
    if type == 1
        f_c = - k_chi * e_chi;
        f_g = - k_gamma * e_gamma;
    elseif type == 2
        f_c = - k_chi * sin(e_chi);
        f_g = - k_gamma * sin(e_gamma);
    elseif type == 3
        f_c = - k_chi * tan(e_chi);
        f_g = - k_gamma * tan(e_gamma);
    end

    f_chi = f_c - chi_c_dot;
    f_gamma = f_g - gamma_c_dot;

end

%-----------------------------------------------------------
% obtain n_lf by gamma_dot and phi,Vg
function n_lf = obtain_n_lf(gamma_dot,Vg,gamma,phi,P)
    g = P.gravity;
    a_zc = gamma_dot * Vg + g * cos(gamma);
    n_lf = a_zc/g/cos(phi);
end

%-----------------------------------------------------------
% obtain reference n_lf and phi according to a_yc and a_zc
function [n_lf_c,phi_c] = obtain_phi_n_c(a_yc,a_zc,P)
    g = P.gravity;
    
    phi_c = asin(a_yc/g);
    n_lf_c = a_zc/g/cos(phi_c);
end

%-----------------------------------------------------------
% obtain gamma_c according to n_lf
function gamma_c = obtain_gamma_c(n_lf_c,n_lf,t,P)
    % commanded flight path angle, with saturation and low pass filter
    gamma_c = PID_gam(n_lf_c - n_lf,P.gam_max,t,P);
end

%-----------------------------------------------------------
% saturation function
function x_sat=sat(x,limit)
  if     x>=limit,  x_sat=limit;
  elseif x<=-limit, x_sat=-limit;
  else              x_sat=x;
  end
end

%-----------------------------------------------------------
% wrap angle with respect to chi
function angle_wrapped=wrap(angle, chi)
  angle_wrapped = angle;
  while (angle_wrapped - chi < -pi), angle_wrapped = angle_wrapped + 2*pi; end
  while (angle_wrapped - chi > +pi), angle_wrapped = angle_wrapped - 2*pi; end
end

%-------------------------------------------------------------
% PID loop for roll angle
function u = PID_phi(error, limit, t, P)
  
    kp = 1;
    kd = 0.5;
    ki = 0.1;

    persistent integrator;
    persistent differentiator;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if t<=P.Ts,
        integrator        = 0; 
        differentiator    = 0;
        error_d1          = 0; 
    end
  
    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
    % update the differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
        + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
    % proportional term
    up = kp * error;
  
    % integral term
    ui = ki * integrator;
  
    % derivative term
    ud = kd * differentiator;
  
  
    % implement PID control
    u = sat(up + ui + ud, limit);
  
    % update persistent variables
    error_d1 = error;
end

%-------------------------------------------------------------
% PID loop for pitch angle
function u = PID_gam(error, limit, t, P)
  
    kp = 10;
    kd = 1;
    ki = 0.1;

    persistent integrator;
    persistent differentiator;
    persistent error_d1;
    % initialize persistent variables at beginning of simulation
    if t<=P.Ts,
        integrator        = 0; 
        differentiator    = 0;
        error_d1          = 0; 
    end
  
    % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
    % update the differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
        + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
    % proportional term
    up = kp * error;
  
    % integral term
    ui = ki * integrator;
  
    % derivative term
    ud = kd * differentiator;
  
  
    % implement PID control
    u = sat(up + ui + ud, limit);
  
    % update persistent variables
    error_d1 = error;
end
