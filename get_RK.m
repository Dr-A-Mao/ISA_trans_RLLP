function RK = get_RK(QK_star)
    RK = 1/max(eig(QK_star));
end