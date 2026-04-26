function [x, y] = ll2local(lat0, lon0, lat, lon, varargin)
% LL2LOCAL 将经纬度坐标转换为以(lat0, lon0)为原点(0,0)的局部平面坐标系
% 输入参数:
%   lat0   - 起点纬度 (度°)，数值/标量
%   lon0   - 起点经度 (度°)，数值/标量
%   lat    - 待转换点的纬度 (度°)，标量/向量/矩阵
%   lon    - 待转换点的经度 (度°)，标量/向量/矩阵（需与lat维度一致）
%   varargin - 可选参数：'km' 输出千米，默认输出米
% 输出参数:
%   x      - 局部坐标系X坐标（东向），米/千米
%   y      - 局部坐标系Y坐标（北向），米/千米
%
% 原理说明：
%   地球近似为球体（半径6371000米），局部范围下忽略曲率，将经纬度差转换为平面距离：
%   - 纬度1° ≈ 111319.9米（固定）
%   - 经度1° ≈ 111319.9*cos(lat0) 米（随纬度变化）
%
% 适用场景：局部范围（<100km）的经纬度转平面坐标，大范围建议使用UTM投影

% 输入参数校验
if size(lat) ~= size(lon)
    error('待转换的纬度(lat)和经度(lon)维度必须一致！');
end

% 地球平均半径（米）
R = 6371000; 

% 计算经纬度差值（度）
dlat = lat - lat0;
dlon = lon - lon0;

% 转换为弧度（也可直接用sind/cosd，结果一致）
dlat_rad = deg2rad(dlat);
dlon_rad = deg2rad(dlon);
lat0_rad = deg2rad(lat0);

% 计算局部坐标系坐标（米）
% X轴（东向）：经度差对应的距离（受起点纬度影响）
x = R * dlon_rad * cos(lat0_rad);
% Y轴（北向）：纬度差对应的距离（固定）
y = R * dlat_rad;

% 可选：输出千米（输入参数含'km'时）
if ~isempty(varargin) && strcmpi(varargin{1}, 'km')
    x = x / 1000;
    y = y / 1000;
end

end