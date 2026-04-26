%% zb2jwd函数
function [lat,lng]= zb2jwd(x,y, Bounding, mapsize)
%输出纬度，经度，输入横坐标，纵坐标，地图起始纬度，起始经度，地图大小(1200还是3600)
lat= (x-1)/ mapsize+Bounding(1,2);
lng= (y-1)/ mapsize+Bounding(1,1);
end