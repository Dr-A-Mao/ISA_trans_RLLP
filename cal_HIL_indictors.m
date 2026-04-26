function [ME,RMSE] = cal_HIL_indictors(Ref_lon_array, Ref_lat_array, Ref_hei_array, ...
                                       lon_array,lat_array,hei_array)
    errors_distance_array = zeros(1,length(Ref_lon_array));
    for k = 1:length(Ref_lon_array)
        
        waypoint_ref_lon = Ref_lon_array(k);
        waypoint_ref_lat = Ref_lat_array(k);
        waypoint_ref_hei = Ref_hei_array(k);
        
        distance_array = sqrt((waypoint_ref_lon - lon_array).^2 + ...
                                (waypoint_ref_lat - lat_array).^2 + ...
                                (waypoint_ref_hei - hei_array).^2);
         [errors_distance_array(k),~] = min(distance_array);
    end
    ME = mean(errors_distance_array);
    RMSE = std(errors_distance_array);
end