%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawPath(path, S, handle, mode)
    flag = path(1); 
    c   = [path(3); path(4); path(5)];
    psi = path(6);
    gam = path(7);
    rho  = path(8);
    lam  = path(9);

    switch flag,
        case 1,
            q  = [cos(psi)*cos(gam); sin(psi)*cos(gam); -sin(gam)];
            XX = [c(1), c(1)+S*q(1)];
            YY = [c(2), c(2)+S*q(2)];
            ZZ = [c(3), c(3)+S*q(3)];
        case 2,
            N = 100;
            t = [0:.1:6*pi];
            XX = c(1) + rho*cos(lam*t+psi);
            YY = c(2) + rho*sin(lam*t+psi);
            %ZZ = c(3) - t*sin(gam);
            ZZ = c(3) - t*rho*tan(gam);
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'g','LineStyle','-.','LineWidth', 0.5,'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 