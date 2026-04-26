function [out,P,h] = update_UAV_states(pn_c,pe_c,pz_c,P,delta_time,h)
    % 配置环境
    if nargin <= 4
        delta_time = 1;
    end
    if nargin <= 3
        param;
    end
    %% 更新P的参数
    P.pn_c = pn_c;
    P.pe_c = pe_c;
    P.pz_c = pz_c;
    %% 调用simlink仿真
    % 将数据P塞到workspace中
    assignin('base','P',P);
    % 仿真simulink,在[0,stopTime]时间段内仿真
    out = sim('model2_1.slx', 'StopTime', num2str(delta_time));
    %% 更新P的剩余部分
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

    draw_flag = false; % 是否把图像画出来
    if nargin == 6 && draw_flag
        %% 画图显示
        h = draw_out(out,h);
    end
end