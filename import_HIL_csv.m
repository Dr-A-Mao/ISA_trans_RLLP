function [lon_array_,lat_array_,hei_array_] = import_HIL_csv(file_path)    
    
    csv_data = readtable(file_path);
    
    lon_array = csv_data.lon;
    lat_array = csv_data.lat;
    alt_array = csv_data.alt;

    [lon_array_, lat_array_] = ll2local(lat_array(1), lon_array(1), lat_array, lon_array, 'm');

    hei_array_ = alt_array;

end
