function QK_star = get_QK_star(A_K0)
    num_I = length(A_K0);
    type = 'equ';
    if strcmp(type,'equ')
        QK_star = lyap(A_K0',eye(num_I));
    end
end