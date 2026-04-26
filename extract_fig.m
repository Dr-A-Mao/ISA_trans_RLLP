clc;
% 提取
[y_dem,x_dem]  = size(demyuanshi);

x_interval = [128.05, 128.10]; 
y_interval = [44.73, 44.78];

y_array = linspace(y(1),y(end-1),y_dem);
delta_y = (y_array(end-1) - y_array(1))/y_dem;

x_array = linspace(x(1),x(end-1),x_dem);
delta_x = (x(end-1) - x(1))/x_dem;

y_indexs = round( ( y_interval - y(1) )/delta_y ) ;

x_indexs = round( ( x_interval - x(1) )/delta_x ) ;

sub_dem = demyuanshi(y_indexs(1) : y_indexs(2),...
                     x_indexs(1) : x_indexs(2) );

lon_array = linspace(x_interval(1),x_interval(2),x_indexs(2) - x_indexs(1) + 1);
lat_array = linspace(y_interval(1),y_interval(2),y_indexs(2) - y_indexs(1) + 1);

[LON,LAT] = meshgrid(lon_array,lat_array); DEM = sub_dem * 0.2;

mesh(LON,LAT,DEM );
xlabel('Lon (。E)'); ylabel('Lat (。W)'); zlabel('Hei (m)');

save('submap.mat','LON','LAT','DEM','lon_array','lat_array');
