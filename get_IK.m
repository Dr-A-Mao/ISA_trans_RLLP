function IK = get_IK(QK_star,RK,A_K0)
    eig_QK_star = eig(QK_star);
    tau_QK_star = max(eig_QK_star)/min(eig_QK_star);
    sigma_min_AK0 = sqrt(min(eig(A_K0' * A_K0)));
    IK = tau_QK_star/(RK * sigma_min_AK0);
end