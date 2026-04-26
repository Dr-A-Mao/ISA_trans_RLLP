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
function out = path_follow_VF(in,P)
  
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
  alpha   = in(5+NN);
  % beta    = in(6+NN);
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

  gamma = theta - alpha; 

  persistent path_data % store path data to detect when it changes
  persistent spiral_index
  persistent varphi_old
  persistent psiTilde

  % 目标点坐标
  persistent pn_c
  persistent pe_c
  persistent h_c
  
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

  % 关键的对比组对于跟踪点方式
  [pn_c_new,pe_c_new,h_c_new] = select_ref_point(pn,pe,h,...
                                  Va,chi,gamma,...
                                  P.path_ref(:,1),...
                                  P.path_ref(:,2),...
                                  -P.path_ref(:,3));
  if norm([pn_c_new - pn_c, pe_c_new - pe_c, h_c_new - h_c]) >= 1 % 若目标点发生改变 
      pn_c = pn_c_new;
      pe_c = pe_c_new;
      h_c = h_c_new;
  end
  

  % computer alpha1, alpha2 and their gradients.
  switch flag,
      case 1, % follow straight line path specified by c_path, psi_path and gam_path

          % alpha1:  longitudinal plane
          n_lon = [-sin(psi_path); cos(psi_path); 0];
          alpha1 = n_lon'*([pn;pe;-h]-c_path);
          gradAlpha1 = n_lon;

          % alpha2:  lateral plane
          q = [cos(psi_path)*cos(gam_path); sin(psi_path)*cos(gam_path); -sin(gam_path)];
          n_lat = cross(q, n_lon);
          alpha2 = n_lat'*([pn;pe;-h]-c_path);
          gradAlpha2 = n_lat;

          % parameters to tune.  We seem to need different values for lines
          % and spirals.  Not sure why.
          k_phi   = 0.3;
          k_grad  = 15*diag([1,1,1]);
          k_cross = 150;
          % compute the commanded velocity vector
          %V = 0.5*(alpha1^2+alpha2^2);
          gradV = alpha1*gradAlpha1 + alpha2*gradAlpha2;
          u = -k_grad*gradV + k_cross*cross(gradAlpha1,gradAlpha2);
          % noralize to ensure velocity vector has length Va_d
          if norm(u)~=0,
              u = Va_d*u/norm(u);
          end

              
      case 2, % follow spiral specified by c_path, rho_ph, gam_path, lambda_path
          
          % alpha1: cylinder
          alpha1 = ((pn-c_path(1))/rho_path)^2 + ((pe-c_path(2))/rho_path)^2 - 1;
          gradAlpha1 = [2*(pn-c_path(1))/rho_path; 2*(pe-c_path(2))/rho_path; 0];

          % alpha2: spiral plane
          d = max((pn-c_path(1))^2+(pe-c_path(2))^2,.1); % distance squared from spiral center
          % angular position on spiral, with logic to unwrap the angles
          varphi = atan2(pe-c_path(2),pn-c_path(1))-psi_path;      
          if varphi-varphi_old>pi,  spiral_index = spiral_index-1; end
          if varphi-varphi_old<-pi, spiral_index = spiral_index+1; end
          varphi_old = varphi;
          % definition of alpha2 and gradient
          tmp = lambda_path*rho_path*tan(gam_path);
          alpha2 = (h+c_path(3)) - tmp*(varphi + 2*pi*spiral_index);
          gradAlpha2 = -[-tmp*(pe-c_path(2))/d; tmp*(pn-c_path(1))/d; 1];
    
          % parameters to tune.  We seem to need different values for lines
          % and spirals.  Not sure why.
          k_phi   = 1;%1;
          k_grad  = 60*diag([1,1,1]);
          k_cross = 150;
          
          % compute the commanded velocity vector
          %V = 0.5*(alpha1^2+alpha2^2);
          gradV = alpha1*gradAlpha1 + alpha2*gradAlpha2;
          u = -k_grad*gradV + lambda_path*k_cross*cross(gradAlpha1,gradAlpha2);
          % noralize to ensure velocity vector has length Va_d
          if norm(u)~=0,
              u = Va_d*u/norm(u);
          end
          
  end
 

  % conversion logic from velocity vector to control signals Va_c, gamma_c,
  % phi_c
  
  % command airspeed equal to desired airspeed
  Va_c = Va_d;

  % commanded flight path angle, with saturation and low pass filter
  gamma_c = -asin(u(3)/Va_d) + PID_gam(-asin(u(3)/Va_d)-theta,P.gam_max,t,P);
  
  % desired heading angle
  psi_c = wrap(atan2(u(2),u(1)), chi);
  % map heading command to roll command  
  psiTilde = wrap(psi_c-chi,psiTilde);
  
  % phi_c comes from PID on psiTilde (HACK)
  phi_c = lambda_path*atan(Va_d^2/P.gravity/rho_path) + k_phi*PID_phi(psiTilde, 45*pi/180, t, P);
      
  % create output
  out = [Va_c; gamma_c; phi_c; alpha2; 0; 0; 0; 0; pn_c;pe_c;h_c];

  % out = [Va_c; gamma_c; phi_c; alpha2; n_lf_c; n_lf;a_yc;a_zc;pn_c;pe_c;h_c];
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




