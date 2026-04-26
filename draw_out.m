%% 根据out画图
function h = draw_out(out,h)
    if nargin == 1
        h = figure;
    end
    figure(h);hold on; grid on; box on;legend off; axis equal;
    xlabel('North(m)');ylabel('East(m)');zlabel('Down(m)');
    plot3(out.pn.Data(2:end),out.pe.Data(2:end),out.pd.Data(2:end),'b-','LineWidth',1.5,'DisplayName','Trajectory');
    plot3(out.pn.Data(2),out.pe.Data(2),out.pd.Data(2),'r+','LineWidth',1,'DisplayName','Start');
    plot3(out.pn.Data(end),out.pe.Data(end),out.pd.Data(end),'gx','LineWidth',1,'DisplayName','End');
    % legend(); %% 画body图
    plot3(out.pn_c.Data,out.pe_c.Data,out.pz_c.Data,'k.','LineWidth',1.5,'DisplayName','Target');
    % 画出有机体的轨迹
    load V_F_colors.mat;
    % 画出机体形状
    num_UAVs = 2;
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
                               pe,pn,-pd,phi,theta, pi/2 -psi,...
                               [], 'normal');
    end
end