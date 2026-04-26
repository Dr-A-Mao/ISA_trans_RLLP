function [ME,RMSE,MAXE,MINE] = get_path_error_indictors(pn_array,pe_array,pz_array, ...
                                              pn_c_array,pe_c_array,pz_c_array)

    d_errors = zeros(1,length(pn_c_array));
  
    for j = 1:length(pn_c_array)
        pn_c = pn_c_array(j); 
        pe_c = pe_c_array(j); 
        pz_c = pz_c_array(j);
        
        d_array = sqrt(( pn_array - pn_c).^2 + ...
                         ( pe_array - pe_c).^2 + ...
                         ( pz_array - pz_c).^2);

        d_min = min(d_array);

        d_errors(j) = d_min;
    end

    ME = mean(d_errors);
    RMSE = std(d_errors);

    MAXE = max(d_errors);
    MINE = min(d_errors);
end
