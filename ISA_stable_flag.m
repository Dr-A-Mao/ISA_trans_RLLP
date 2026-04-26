%-----------------------------------------------------------
% 保证稳定的参数选择
function ISA_flag = ISA_stable_flag(k_pi,k_d,k_w,v_max,v_min)
    
    if nargin == 3
        v_max = 15;
        v_min = 9;
    end
    
    Eq1 = sqrt(k_pi * k_d * v_max/k_w) - pi/4;

    Eq2 = v_min * sin(sqrt(k_pi * k_d * v_max/k_w)) + v_max - 2/pi * v_min * k_pi * cos(sqrt(k_pi * k_d * v_max/k_w));

    if (Eq1 <= 0) && (Eq2 < 0)
        ISA_flag = true;
    else
        ISA_flag = false;
    end

end