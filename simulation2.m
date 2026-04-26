%% 更新参数数据并仿真
function [P,out] = simulation2(P,stopTime)
    %% 将数据P塞到workspace中
    assignin('base','P',P);
    %% 仿真simulink,在[0,stopTime]时间段内仿真
    out = sim('model2.slx', 'StopTime', num2str(stopTime));
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