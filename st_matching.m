clear;clc;
%% cutting road network to grid
roadnetworkfilename = 'C:\Users\MiaoYu\Documents\我的坚果云\时空轨迹数据_FiveCities\BeijingCity\RoadNetwork_Beijing.txt';
cell_size = 1;
% [road_network,road_cells,grid_size] = splitRoad2Cell(roadnetworkfilename, cell_size);
load('road&cell.mat');grid_size = [74 92];
%% splitting GPS trajactories
% gpsfilename = 'C:\Users\MiaoYu\Documents\我的坚果云\时空轨迹数据_FiveCities\BeijingCity\GPS_Beijing.txt';
% raw_gps_points = splitGPS2line(gpsfilename, 6, 5);
load('GPS_Points.mat')
raw_gps_points = GPSBeijing;
clear GPSBeijing
%% fetching GPS candidate points
% single trajactories
search_radius = 1;
trajactory = raw_gps_points(raw_gps_points.Tag == 2 ,:);
trajactory.CandidatePoints = cell(height(trajactory),1);
trajactory.CandidateEdges = cell(height(trajactory),1);
for point_idx = 1:height(trajactory)
    lon = trajactory.Longitude(point_idx);
    lat = trajactory.Latitude(point_idx);
    [road_ids, candPoints] = getCandidatePoints(lon,lat,...
        road_network,road_cells,search_radius,cell_size,grid_size);
    [row_can,~] = size(candPoints);
    trajactory.CandidatePoints(point_idx) = mat2cell(candPoints,row_can);
    trajactory.CandidateEdges(point_idx) = mat2cell(road_ids,row_can);
end
clearvars candPoints lat lon point_idx roadnetworkfilename row_can
[G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_network,grid_size);
