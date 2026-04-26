%% 更新参数数据并仿真
function [P,out] = simulation(ref_t,ref_Va_c,ref_gamma_c,ref_phi_c,P_origin)
    stopTime = 5; %仿真时间
    P = P_origin; % 原始的P
    P.ref_data = input_ref_signals(ref_t,ref_Va_c,ref_gamma_c,ref_phi_c);
    %% 将数据P塞到workspace中
    assignin('base','P',P);
    %% 仿真simulink,在[0,stopTime]时间段内仿真
    out = sim('model.slx', 'StopTime', num2str(stopTime));
    %% 更新P值
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
end