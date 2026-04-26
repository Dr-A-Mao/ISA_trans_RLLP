function  t = get_replanning_time(ucav,X2,Pc)
    Vg = ucav.Vg;
    X1 = [ucav.xr,ucav.yr,ucav.zr];

    gamma1_c = asin((Pc(3) - ucav.zr)/norm([Pc(2) - ucav.yr, Pc(1) - ucav.xr]));
    chi1_c = atan2(Pc(2) - ucav.yr,Pc(1) - ucav.xr);

    gamma2_c = asin((X2(3) - Pc(3))/norm([X2(2) - Pc(2), X2(1) - Pc(1)]));
    chi2_c = atan2(X2(2) - Pc(2),X2(1) - Pc(1));

    eta_lon1 = gamma1_c - ucav.gamma;
    eta_lat1 = chi1_c - ucav.chi;

    eta_lon2 = gamma2_c - gamma1_c;
    eta_lat2 = chi2_c - chi1_c;

    if abs(eta_lon1) >= pi/2 || abs(eta_lat1) >= pi/2 ||...
            abs(eta_lon2) >= pi/2 || abs(eta_lat2) >= pi/2
        t = inf;
    else
        t1 = norm(Pc - X1)/(Vg * cos(eta_lon1) * cos(eta_lat1));
        t2 = norm(X2 - Pc)/(Vg * cos(eta_lon2) * cos(eta_lat2));
        t = t1 + t2;
    end
end