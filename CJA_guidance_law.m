function [phi_d,theta_d] = CJA_guidance_law(xi,yj,zk,Va,psi,d_hat,P,curvetype,C,l)
    if nargin == 7
        curvetype = 'lissajous_CJA';
        C = 2.7825e+03; % 总周长
        l = 0; % UAV当前的飞行轨迹长度
    end
    %% 声明关键参数
    rho = 0.1;
    k = [0.5, 0.5, 0.1];
    if strcmp(curvetype,'lissajous_CJA')
        North_range = 320; North_bais = 300;
        East_range = 280; East_bais = 300;
        Height_range = 50; Height_bais = 50;
        numbers = 80;

        tau = 20 * pi/C;
        
        t_array = linspace(0, 20 * pi, numbers);  % 时间参数
        A = [North_range, East_range, Height_range]; 
        phi = [pi/2, 0, pi/2];         % x, y, z方向相位
        f = [-0.1, -0.2, -0.2];        % x, y, z方向频率

        t0 = t_array(P.init_ref_ind);

        f1 = @(t) A(1) * sin(f(1)*( t + t0 ) + phi(1)) + North_bais;
        f2 = @(t) A(2) * sin(f(2)*( t + t0 ) + phi(2)) + East_bais;
        f3 = @(t) A(3) * sin(f(3)*( t + t0 ) + phi(3)) + Height_bais;

        f1_dot = @(t) A(1) * f(1)* cos(f(1)*( t + t0 ) + phi(1));
        f2_dot = @(t) A(2) * f(2)* cos(f(2)*( t + t0 ) + phi(2));
        f3_dot = @(t) A(3) * f(3)* cos(f(3)*( t + t0 ) + phi(3));
    end

    %% 计算(xc,yc,zc)中对应的wl
    wl = l;

    %% 开始计算制导率
    varphi1 = @(x,y,z,w)(x - f1(w * tau));
    varphi2 = @(x,y,z,w)(y - f2(w * tau));
    varphi3 = @(x,y,z,w)(z - f3(w * tau));

    xi_d_dot = @(x,y,z,w)[ -rho^3 * tau * f1_dot(tau * w) - k(1) * rho^2 * varphi1(x,y,z,w);...
                   -rho^3 * tau * f2_dot(tau * w) - k(2) * rho^2 * varphi2(x,y,z,w);...
                   -rho^3 * tau * f3_dot(tau * w) - k(3) * rho^2 * varphi3(x,y,z,w);...
                   -rho^3 + rho^2 * tau * ( k(1) * varphi1(x,y,z,w) * f1_dot(tau * w) + ...
                                            k(2) * varphi2(x,y,z,w) * f2_dot(tau * w) + ...
                                            k(3) * varphi3(x,y,z,w) * f3_dot(tau * w)  )];
    % wl根据xc,yc,zc选择出来
    xi_d_dot_value = xi_d_dot(xi,yj,zk,wl);
    Vg_d_bar = xi_d_dot_value(1:3)/norm(xi_d_dot_value(1:3));
    P_d_dot_bar = Vg_d_bar;
    % 计算b,c
    b = P_d_dot_bar' * d_hat / Va;
    c = norm(d_hat) / Va;
    % 计算s_star
    s_star = b + sqrt(b^2 - c^2 + 1);
    % 计算v_1_d
    v_1_d = s_star * P_d_dot_bar - d_hat/Va; % v_1_d的norm为1
    % 计算phi_d, theta_d
    theta_d = asin(v_1_d(3));
    if theta_d >= pi/6
        theta_d = pi/6;
    elseif theta_d <= -pi/12
        theta_d = -pi/12;
    end
    % 计算varepsilon
    varepsilon = [cos(psi),sin(psi)]';
    % 计算psi_d_dot
    varepsilon_d = v_1_d(1:2)/norm(v_1_d(1:2));
    % 计算当前的psi_d
    psi_d = atan2(varepsilon_d(2),varepsilon_d(1));
    varepsilon_tilde = [-sin(psi_d),cos(psi_d)]';
    E = [0, 1; -1, 0];
    psi_d_dot =  varepsilon_d' * E * (-varepsilon)/(1 - varepsilon_d' * E * varepsilon_tilde ) ;
    phi_d = atan2( Va * psi_d_dot , P.gravity);
    if phi_d >= pi/4
        phi_d = pi/4;
    elseif phi_d <= -pi/4
        phi_d = -pi/4;
    end

end