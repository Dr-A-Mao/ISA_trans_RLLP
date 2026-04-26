% path follow
%  - follow straight line path or orbit
%
% Modified:
%   3/25/2010  - RB
%   6/5/2010   - RB
%   11/08/2010 - RB
%   4/4/2013   = RB
%
% input is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%   xhat   - estimated MAV states (pn, pe, pd, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi)
%
% output is:
%  Va_c - airspeed command
%  h_c  - altitude command
%  chi_c - heading command
%
function out = path_follow_AST(in,P)
  %% 根据dubins路径给出的参考状态量
  % 同时也是JGCD的
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
  persistent pn_c_old
  persistent pe_c_old
  persistent t_old
  
  % logic to ensure smooth switching to new spiral and straight line paths
  if t<P.Ts,
      path_data = [flag; Va_d; c_path; psi_path; gam_path; rho_path; lambda_path];
      spiral_index = 0;
      varphi_old = 0;
      psiTilde   = 0;
      pn_c_old = 0;
      pe_c_old = 0;
      t_old = 0;
  else
      path_data_new = [flag; Va_d; c_path; psi_path; gam_path; rho_path; lambda_path];
      if norm(path_data-path_data_new)>.1,  % if the path has changed, then reset the spiral angle
          path_data = path_data_new;
          spiral_index = 0;
          varphi_old = 0;
      end 
  end
  
  % computer alpha1, alpha2 and their gradients.
  switch flag
      case 1 % follow straight line path specified by c_path, psi_path and gam_path

          % alpha1:  longitudinal plane
          n_lon = [-sin(psi_path); cos(psi_path); 0];

          % alpha2:  lateral plane
          q = [cos(psi_path)*cos(gam_path); sin(psi_path)*cos(gam_path); -sin(gam_path)];
          n_lat = cross(q, n_lon);
          alpha2 = n_lat'*([pn;pe;-h]-c_path);
         
          % obtain n_lf
          n_lf = obtain_n_lf(gamma_dot,Va,gamma,phi,P); n_lf_c = 0; I_f = 1;
          
          % 关键的对比组对于跟踪点方式
          [pn_c,pe_c,h_c] = select_ref_point(pn,pe,h,...
                                          Va,chi,gamma,...
                                          P.path_ref(:,1),...
                                          P.path_ref(:,2),...
                                          -P.path_ref(:,3));
         
          %disp(['pn_c:',num2str(pn_c),',pe_c:',num2str(pe_c),',h_c:',num2str(h_c)]);
          theta = atan2( pe_c - pe, pn_c - pn); % LOS angle theta
            
          xi = theta - chi;
          r = norm([pe_c - pe, pn_c - pn]);

          g = 9.81;
          M1 = 20; N1 = 5;  alpha1 = 1.01; beta1 = 0.9;
          M2 = 50; N2 = 25; alpha2 = 1.01; beta2 = 0.9; 

          psi_r = atan2(pe_c - pe_c_old, pn_c - pn_c_old);
          v_r = norm([pe_c - pe_c_old, pn_c - pn_c_old])/(t - t_old);
          if isnan(v_r) || isinf(v_r)
              v_r = Va_c;
          end

          w_x = P.wind_n;
          w_y = P.wind_e;
          v_w = norm([w_x,w_y]);

          Vg = (v_r * cos(theta - psi_r) + M1 * r^alpha1 + N1 * r^beta1)/cos(theta - chi);
          
          wg = v_r/r * (tan(xi) * cos(theta - psi_r) - sin(theta-psi_r)) +...
               tan(xi)/r * (M1 * r^alpha1 + N1 * r^beta1) +...
               (M2 * abs(xi)^alpha2 + N2 * abs(xi)^beta2) * sign(xi);
          if isnan(wg)
            wg = 0;
          end

          Va_c = sqrt(Vg^2-2*Vg*(w_x*cos(chi) + w_y *sin(chi))+v_w^2);
         
          psi_c = chi + (t - t_old) * wg;

          % 根据psi_c得到phi_c
          psiTilde = wrap(psi_c-chi,psiTilde);
          % phi_c comes from PID on psiTilde (HACK)
          k_phi = 0.3;
          phi_c = lambda_path*atan(Va_c^2/P.gravity/rho_path) + k_phi*PID_phi(psiTilde, 45*pi/180, t, P);
      
          gamma_c = atan2( h_c - h, norm([ pe_c - pe , pn_c - pn ]));
  end

  % update the ref pn_c_old, pe_c_old
  pn_c_old = pn_c; pe_c_old = pe_c; t_old = t;
  % create output
  out = [Va_c; gamma_c; phi_c; I_f; n_lf_c; n_lf;0;0;pn_c;pe_c;h_c];
end

%-----------------------------------------------------------
% select reference points
function [x_p,y_p,z_p] = select_ref_point(x_r,y_r,z_r,Vg_r,chi_r,gamma_r,...
                                          path_x,path_y,path_z)
    % 路径参量
    path_eta_lat = atan2(path_y - y_r,path_x - x_r) - chi_r;
    path_eta_lon = atan2(path_z - z_r, sqrt((path_x - x_r).^2 + (path_y - y_r).^2)) - gamma_r;
    path_index =  find((abs(path_eta_lat) <= pi/3) & (abs(path_eta_lon) <= pi/2));
    if ~isempty(path_index)
        path_x = path_x(path_index);
        path_y = path_y(path_index);
        path_z = path_z(path_index);
    end
    % 搜索在路径path中最近投影点
    distance_r_path = sqrt((path_x-x_r).^2 + ...
                           (path_y-y_r).^2 + ...
                           (path_z-z_r).^2);
    [~,i] = min(distance_r_path); % 投影点索引为i
    next_distance_array = distance_r_path(i:end);
    % 设计期望的周期P_L和阻尼xi_L，q_L，计算重要参数
    xi_L = 0.707; P_L = 10;
    % 下面计算重要的制导率
    L = P_L * xi_L /pi * Vg_r;
    %% 需要修改为待跟踪的轨迹
    % 计算下一点的索引
    [~,k] = min(abs(next_distance_array - L));
    %待跟踪结点索引 
    p_index = i + k - 1;
    % disp(['i:',num2str(i),',p_index:',num2str(p_index)]);
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
    
    %% 根据chi_c进行修正chi_c（极大的创新点）!!!!
    if eta_lat < -pi
        if chi_c <= 0 % 逆时针转动过界了
            chi_c = pi - chi_c;
            eta_lat = chi_c - chi;
        end
    elseif eta_lat > pi
       if chi_c >= 0 % 逆时针转动过界了 
            chi_c = -pi - chi_c;
            eta_lat = chi_c - chi;
       end
    end
    %% 函数类别
    [f_chi,f_gamma] = f(eta_lat,eta_lon,chi_c_dot,gamma_c_dot,P,type);

    a_yc = - Vg * cos(phi) * f_chi;
    a_zc = - Vg * f_gamma + g * cos(gamma);

%     disp(['chi_c:',num2str(chi_c * 180/pi),'deg,' ...
%           'chi:',num2str(chi * 180/pi),'deg,' ...
%           'eta_lat:',num2str(eta_lat * 180/pi),'deg,' ...
%           'eta_lon:',num2str(eta_lon * 180/pi),'deg,' ...
%           'a_yc:',num2str(a_yc),',a_zc:',num2str(a_zc)]);
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
    elseif type == 4
        f_c = 1 - exp( k_chi * e_chi );
        f_g = 1 - exp( k_gamma * e_gamma );
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
