disp(['----位于param.m中的函数开始初始化----']);
close all; warning off;
%% 用于测试用的
if false
    clear;
end

if false
    P.gamma_c = pi/10;
    P.n_lf_c = 1;
    P.phi_c = 0;

    P.pn_c = 70;
    P.pe_c = 40;
    P.pz_c = 50;
end

P.Va_c = 13;
P.gravity = 9.81;

% 采用的pathfollowing方法
% P.method = 'VF-Line'; 
% P.method = 'VF-Dubins'; 
% P.method = 'ISA_law'; 
% P.method = 'CJA_law'; 
% P.method = 'AST_law';
% P.method = 'Optimal-Analytical';
P.method = 'RLLP-sin(x)';
% P.method = 'L1_law';

% 风场扰动的类型
P.wind_type = 'w0';

% RLLP的控制器参数
P.k_type = 'k1';

% 待跟随的pathfollowing的曲线
if strcmp(P.method,'VF-Line') ||...
   strcmp(P.method,'VF-Dubins') ||...
   strcmp(P.method,'AST_law')
     P.path_type = 'line';
else
    P.path_type = 'lissajous_CJA';
end
% P.path_type = 'lissajous';
P.path_type = 'line';
% P.path_type = 'ellipse';
% P.path_type = 'spiral';
% P.path_type = 'lissajous_CJA';
% P.path_type = 'helix_CJA';
% P.path_type = 'circle';
% P.path_type = 'SIL';

% VF-Dubins, VF-Line, RLLP-x, RLLP-sin(x), RLLP-tan(x), Optimal-RLLP
if strcmp(P.method, 'VF-Dubins')
    P.algorithm = 'VF-Dubins';
    P.type_dubins = true;
elseif strcmp(P.method, 'VF-Line')
    P.algorithm = 'VF-Line';
    P.type_dubins = false;
elseif strcmp(P.method, 'RLLP-x')
    P.algorithm = 'RLLP';
    P.type_dubins = false; 
    P.f_type = 1;
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'RLLP-sin(x)')
    P.algorithm = 'RLLP';
    P.type_dubins = false; 
    P.f_type = 2;
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'RLLP-tan(x)')
    P.algorithm = 'RLLP';
    P.type_dubins = false; 
    P.f_type = 3;
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'RLLP-exp(x)') 
    P.algorithm = 'RLLP';
    P.type_dubins = false; 
    P.f_type = 4;
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'Optimal-Robustness') % 最快速率
    P.algorithm = 'Optimal-RLLP';
    P.optimization_type = 'robustness';
    P.type_dubins = false; 
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'Optimal-Energy') % 最小能量
    P.algorithm = 'Optimal-RLLP';
    P.optimization_type = 'energy';
    P.type_dubins = false; 
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'Optimal-Attractor') % 最小吸引子
    P.algorithm = 'Optimal-RLLP';
    P.optimization_type = 'attractor';
    P.type_dubins = false; 
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'Optimal-Analytical') % 最小吸引子
    P.algorithm = 'Optimal-RLLP';
    P.optimization_type = 'analytical';
    P.type_dubins = false; 
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.method, 'AST_law')
    P.algorithm = 'AST_law';
    P.type_dubins = false;
elseif strcmp(P.method, 'ISA_law')
    P.algorithm = 'ISA_law';
    P.type_dubins = false;   
elseif strcmp(P.method, 'CJA_law')
    P.algorithm = 'CJA_law';
    P.type_dubins = false;  
elseif strcmp(P.method, 'L1_law')
    P.algorithm = 'L1_law';
    P.type_dubins = false; 
end

% wind分级
if strcmp(P.wind_type,'w1')
    P.wind_n = 5;
    P.wind_e = 0;
    P.wind_d = 0;
    P.U_A = 0;
    P.V_A = 0;
    P.W_A = 0;
elseif strcmp(P.wind_type,'w2')
    P.wind_n = 5;
    P.wind_e = 5;
    P.wind_d = 0;
    P.U_A = 0;
    P.V_A = 0;
    P.W_A = 0;
elseif strcmp(P.wind_type,'w3')
    P.wind_n = 5;
    P.wind_e = 5;
    P.wind_d = 2;
    P.U_A = 0;
    P.V_A = 0;
    P.W_A = 0;
elseif strcmp(P.wind_type,'w4')
    P.wind_n = 10;
    P.wind_e = 10;
    P.wind_d = 2;
    P.U_A = 0;
    P.V_A = 0;
    P.W_A = 0;
elseif strcmp(P.wind_type,'w5') 
    P.wind_n = 5;
    P.wind_e = 5;
    P.wind_d = 0;
    P.U_A = 0.25;
    P.V_A = 0.125;
    P.W_A = 0.125;
elseif strcmp(P.wind_type,'w6') 
    P.wind_n = 5;
    P.wind_e = 5;
    P.wind_d = 0;
    P.U_A = 0.5;
    P.V_A = 0.25;
    P.W_A = 0.25;
elseif strcmp(P.wind_type,'w0')
    P.wind_n = 0;
    P.wind_e = 0;
    P.wind_d = 0;
    P.U_A = 0;
    P.V_A = 0;
    P.W_A = 0;
elseif strcmp(P.wind_type,'w7')
    P.wind_n = 10;
    P.wind_e = 10;
    P.wind_d = 5;
    P.U_A = 0;
    P.V_A = 0;
    P.W_A = 0;    
end


% I(f)分级
if strcmp(P.k_type,'k1')
    P.k_chi = 0.5;
    P.k_gamma = 0.5;
elseif strcmp(P.k_type,'k2')
    P.k_chi = 0.5;
    P.k_gamma = 1;
elseif strcmp(P.k_type,'k3')
    P.k_chi = 1;
    P.k_gamma = 0.5;
elseif strcmp(P.k_type,'k4')
    P.k_chi = 1;
    P.k_gamma = 1;
elseif strcmp(P.k_type,'k5')
    P.k_chi = 2;
    P.k_gamma = 2;
elseif strcmp(P.k_type,'k6')
    P.k_chi = 4;
    P.k_gamma = 4;
elseif strcmp(P.k_type,'k7')
    P.k_chi = 0.25;
    P.k_gamma = 0.25;
elseif strcmp(P.k_type,'k8')
    P.k_chi = 0.1;
    P.k_gamma = 0.1;
end

K = 0.1;
L = 15;  % 离散点的数目
h0 = 40; % UAV初始高度

P.out_path = ['Exp_',P.wind_type,'_',P.k_type];

%import reference path
load('path.mat');

lng_resolution = 110e3;
lat_resolution = 113e3;

P.init_lng = path_lng(1);
P.init_lat = path_lat(1);

P.K = K;

if strcmp(P.path_type, 'line')
    % 输出路径
    
    path_lng = (path_lng - path_lng(1)) * lng_resolution * K;
    path_lat = (path_lat - path_lat(1)) * lat_resolution * K;
    path_hei = (path_hei - path_hei(1)) * K + h0;
    
    path_series = round(linspace(1,length(path_lat),L));
    
    P.path_ref = [path_lat(path_series),...
                  path_lng(path_series),...
                  -path_hei(path_series)];
    P.pn0    = 0;  % initial North position
    P.pe0    = 0;  % initial East position
    P.pd0    = -h0;  % initial Down position (negative altitude)
elseif strcmp(P.path_type, 'lissajous')
    load('lissajous.mat');
    P.path_ref = path_ref;
    P.pn0    = 0;  % initial North position
    P.pe0    = 0;  % initial East position
    P.pd0    = -h0;  % initial Down position (negative altitude)
elseif strcmp(P.path_type, 'ellipse')
    load('ellipse.mat');
    P.path_ref = path_ref;
    P.pn0    = 0;  % initial North position
    P.pe0    = 0;  % initial East position
    P.pd0    = -h0;  % initial Down position (negative altitude)
elseif strcmp(P.path_type, 'spiral')
    load('spiral.mat');
    P.path_ref = path_ref;
    P.pn0    = 0;  % initial North position
    P.pe0    = 0;  % initial East position
    P.pd0    = -h0;  % initial Down position (negative altitude)
elseif strcmp(P.path_type, 'lissajous_CJA')
    load('lissajous_CJA.mat');
    P.path_ref = path_ref;
    if strcmp(P.method,'RLLP-sin(x)') ||...
        strcmp(P.method,'CJA_law')
        P.init_ref_ind = 21;
    elseif strcmp(P.method,'AST_law')
        P.init_ref_ind = 32;
    end
    P.pn0    = P.path_ref(P.init_ref_ind,1);  % initial North position
    P.pe0    = P.path_ref(P.init_ref_ind,2);  % initial East position
    P.pd0    = P.path_ref(P.init_ref_ind,3);  % initial Down position (negative altitude)
    % 如果是RLLP算法，则需要对结点进行额外waypoints进行引导
    if strcmp(P.method,'RLLP-sin(x)')
        path_ref_61_point = P.path_ref(61,:);
        path_ref_62_point = P.path_ref(62,:);

        path_ref_61_1_3_point = path_ref_61_point + (path_ref_62_point - path_ref_61_point) * 1/3;
        path_ref_61_2_3_point = path_ref_61_point + (path_ref_62_point - path_ref_61_point) * 2/3;

        P.path_ref = [P.path_ref(1:61,:);...
                      path_ref_61_1_3_point;...
                      path_ref_61_2_3_point;...
                      P.path_ref(62:end,:)];
    end

elseif strcmp(P.path_type, 'helix_CJA')
    load('helix_CJA.mat');
    P.path_ref = path_ref;
    P.pn0    = P.path_ref(1,1);  % initial North position
    P.pe0    = P.path_ref(1,2);  % initial East position
    P.pd0    = P.path_ref(1,3);  % initial Down position (negative altitude)
elseif strcmp(P.path_type, 'circle')
    load('circle.mat'); h0 = 48;
    P.path_ref = [path_ref(:,1:2), -h0 * ones(size(path_ref(:,1)))];
    P.pn0    = 0;  % initial North position
    P.pe0    = 0;  % initial East position
    P.pd0    = -h0;  % initial Down position (negative altitude)
    % 声明扰动幅值和范围
    P.wind_n = 8;
    P.wind_e = 0;
    P.wind_d = 0;
elseif strcmp(P.path_type, 'SIL')
    load('SIL.mat');
    P.path_ref = path_ref;
    P.pn0    = P.path_ref(1,1);  % initial North position
    P.pe0    = P.path_ref(1,2);  % initial East position
    P.pd0    = P.path_ref(1,3);  % initial Down position (negative altitude)
end

%physical parameters of airframe
P.mass = 1.56;
P.Jx   = 0.1147;
P.Jy   = 0.0576;
P.Jz   = 0.1712;
P.Jxz  = 0.0015;

% aerodynamic coefficients
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
P.rho           = 1.2682;
P.c             = 0.3302;
P.b             = 1.4224;
P.S_wing        = 0.2589;
P.S_prop        = 0.0314;
P.k_motor       = 20;
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_M_0         = 0.0;
P.C_M_alpha     = -0.38;
P.C_M_q         = -3.6;
P.C_M_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = -0.26;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1;


% wind parameters
P.L_wx = 1250;
P.L_wy = 1750;
P.L_wz = 1750;
P.sigma_wx = 1; 
P.sigma_wy = 1;
P.sigma_wz = 0.7;
% P.sigma_wx = 0; 
% P.sigma_wy = 0;
% P.sigma_wz = 0;
P.Va0 = 13;

% autopilot sample rate
P.Ts = 0.01;

% compute trim conditions using 'mavsim_chap5_trim.mdl'
P.Va    = 13;         % desired airspeed (m/s)
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = 0;         % desired radius (m) - use (+) for right handed orbit, 
                    %                          (-) for left handed orbit
% first cut at initial conditions

P.u0     = P.Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0*pi/180;  % initial heading angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
   = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

P.A_lon = A_lon; % (5.49)的A_lon
P.B_lon = B_lon; % (5.49)的B_lon
P.A_lat = A_lat; % (5.42)的A_lat
P.B_lat = B_lat; % (5.42)的B_lat

% gain on dirty derivative
P.tau = 5;

% autopilot gains
% altitude parameters and gains
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 10;
 % 用于获得控制器的参数
computeGains;

% sensor parameters
%P.sigma_gyro = 0.13*pi/180; % standard deviation of gyros in rad/sec
%P.bias_gyro_x = 0.1*pi/180*rand; % bias on x_gyro
%P.bias_gyro_y = 0.1*pi/180*rand; % bias on y_gyro
%P.bias_gyro_z = 0.1*pi/180*rand; % bias on z_gyro
%P.sigma_accel = 0.0025*9.8; % standard deviation of accelerometers in m/s^2
%P.sigma_static_pres = 0.01*1000; % standard deviation of static pressure sensor in Pascals
%P.sigma_diff_pres = 0.002*1000;  % standard deviation of diff pressure sensor in Pascals

% set sensor noise to zero to test KF with no noise (should work great)
% P.sigma_gyro = 0.0; % standard deviation of gyros in rad/sec
% P.bias_gyro_x = 0.0; % bias on x_gyro
% P.bias_gyro_y = 0.0; % bias on y_gyro
% P.bias_gyro_z = 0.0; % bias on z_gyro
% P.sigma_accel = 0.0; % standard deviation of accelerometers in m/s^2
% P.sigma_static_pres = 0.0; % standard deviation of static pressure sensor in Pascals
% P.sigma_diff_pres = 0.0;  % standard deviation of diff pressure sensor in Pascals


% GPS parameters
%P.Ts_gps = 1; % sample rate of GPS in s
% P.beta_gps = 1/1100; % 1/s
% P.sigma_n_gps = 0.21;
% P.sigma_e_gps = 0.21; 
% P.sigma_h_gps = 0.40;
% P.sigma_Vg_gps = 0.05;
% P.sigma_course_gps = P.sigma_Vg_gps/P.Va;


% parameters for guidance model
zeta_chi = .7;
wn_chi   = 0.75;
% zeta_chi = .7;
% wn_chi   = 2;
P.b_chi    = wn_chi^2;
P.b_chidot = 2*zeta_chi*wn_chi;

P.b_Va     = 1;

zeta_h       = 1.1;
wn_h         = .6;
P.b_h      = wn_h^2;
P.b_hdot   = 2*zeta_h*wn_h;
P.a_hdot   = .01;
P.b_hnum   = 0;

P.b_phi    = 1/0.2;


% number of waypoints in data structure
P.size_waypoint_array = 100;
P.R_min = 20;  % minimum turn radius
P.gam_max = 10*pi/180; % maximum flight path angle

% create random city map
city_width      = 500;  % the city is of size (width)x(width)
building_height = 50;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 5;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
%P.pd0           = -30;  % initial height of MAV
map = createWorld(city_width, building_height, num_blocks, street_width);% modify the map
buildings_n = [50,150,250,350,450,550];
buildings_e = [50,150,250,350,450,550];
heights = [ 20.8634, 54.4626, 59.0126, 16.5987,  21.7390, 35;...
            2.4827, 16.8860, 19.4869, 47.1025,  17.6579, 45;...
            45.1358, 45.0027, 12.0846, 47.8067, 5.0597, 45;...
            47.2394, 18.4623, 20.1956, 28.7604,  50.7702, 45;...
            24.5432,  5.5601,  4.8227,  2.9890,  2.1512, 45;...
            25, 35, 35, 35, 35, 5];
map.buildings_n = buildings_n;
map.buildings_e = buildings_e;
map.heights = heights;
% % target parameters
% P.target_velocity = 5;  % (m/s)
% P.target_size = 2;          % size of target 
% 
% 
% % gimbal parameters
% P.az0 = 0;      % initial azimuth angle
% P.el0 = -pi/2;  % initial elevation angle (pointing down)
% P.az_limit = 180*(pi/180);  % azimuth angle limit
% P.el_limit = 180*(pi/180);  % elevation angle limit
% P.az_gain  = 1;  % gain on azimuth dynamics (azdot = az_gain*u_az)
% P.el_gain  = 1;  % gain on elevation dynamics (eldot = el_gain*u_el)
% P.k_az     = 10; % proportional control gain for gimbal azimuth
% P.k_el     = 10; % proportional control gain for gimbal elevation
% 
% % camera parameters
% P.cam_fps = 10;  % frames per second 
% P.cam_pix = 480;                      % size of (square) pixel array
% P.cam_fov   = 10*(pi/180);            % field of view of camera
% P.f = (P.cam_pix/2)/tan(P.cam_fov/2); % focal range
% P.pixelnoise = 0;                     % (pixels) - variance of the pixel noise
% 
% % measurement model for GPS (used in geolocation algorithm)
% P.sigma_measurement_n = 0.547; % standard deviation of gps measurement error in m
% P.sigma_measurement_e = 0.547; % standard deviation of gps measurement error in m
% P.sigma_measurement_h = 1.14; % standard deviation of gps measurement error in m
% 



disp(['----初始化完毕----']);


