clear;clc;
%% cutting road network to grid
roadnetworkfilename = 'C:\Users\MiaoYu\Documents\我的坚果云\时空轨迹数据_FiveCities\BeijingCity\RoadNetwork_Beijing.txt';
cell_size = 0.1;
[road_network,road_cells,grid_size] = splitRoad2Cell(roadnetworkfilename, cell_size);
% load('road&cell.mat');grid_size = [74 92];
%% splitting GPS trajactories
% gpsfilename = 'C:\Users\MiaoYu\Documents\我的坚果云\时空轨迹数据_FiveCities\BeijingCity\GPS_Beijing.txt';
% raw_gps_points = splitGPS2line(gpsfilename, 6, 5);
load('GPS_Points.mat')
raw_gps_points = GPSBeijing;
clear GPSBeijing
%% fetching GPS candidate points
% single trajactories
search_radius = 0.1;
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
%%
[G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_network,grid_size);
%% Find matched sequence start

%first round
cand_points_prev = cell2mat(trajactory.CandidatePoints(1));
cand_edges_prev = cell2mat(trajactory.CandidateEdges(1));
sample_point_prev = [trajactory.Longitude(1) trajactory.Latitude(1)];
f_prev = zeros(1,length(cand_points_prev));
for cand_idx = 1 : length(cand_points_prev)
    f_prev(cand_idx) = fcnNormal(cand_points_prev(cand_idx,:),sample_point_prev);
end
%% second round search
parent_table = table;
parent_table.sample_idx = zeros(0);parent_table.candidate_idx = zeros(0);
parent_table.parent = cell(0);parent_table.parent_idx = zeros(0);
wbh_traj = waitbar(0,'Processing Sampling Points');
for sample_idx = 2:height(trajactory)
    delta_t = trajactory.Timestamp(sample_idx) - trajactory.Timestamp(sample_idx-1);
    sample_point_cur = [trajactory.Longitude(sample_idx),...
        trajactory.Latitude(sample_idx)];
    cand_points_cur = cell2mat(trajactory.CandidatePoints(sample_idx));
    cand_edges_cur = cell2mat(trajactory.CandidateEdges(sample_idx));
    f_cur = zeros(1,length(cand_points_cur));
    wbh_calc = waitbar(0,'Caluculating Scores for this edge');
    warning('off','all');
    for idx_cur = 1 : length(f_cur)
        cur_max = -Inf;
        for idx_prev = 1: length(f_prev)
            alt = f_prev(idx_prev) +  ...
                fcnSpatialTemporal(cand_points_prev(idx_prev,:),...
                cand_points_cur(idx_cur,:),sample_point_prev,sample_point_cur,...
                delta_t,cand_edges_prev(idx_prev),cand_edges_cur(idx_cur),...
                G,node_table,road_network);
            if alt > cur_max
                cur_max = alt;
                parent_loc = find(parent_table.sample_idx==sample_idx & parent_table.candidate_idx == idx_cur);
                if ~any(parent_loc)
                    parent_loc = height(parent_table)+1;
                end
                parent_table.sample_idx(parent_loc) = sample_idx;
                parent_table.candidate_idx(parent_loc) = idx_cur;
                parent_table.parent(parent_loc) = mat2cell(cand_points_prev(idx_prev,:),1);
                parent_table.parent_idx(parent_loc) = idx_prev;
                waitbar((idx_prev+(idx_cur-1)*length(f_prev))/(length(f_prev)*length(f_cur)),wbh_calc);
            end
            f_cur(idx_cur) = cur_max; 
        end
    end
    close(wbh_calc);
    cand_points_prev = cand_points_cur;cand_edges_prev = cand_edges_cur;
    sample_point_prev = sample_point_cur;f_prev = f_cur;
    waitbar(sample_idx/(height(trajactory)-1),wbh_traj);
end
warning('on','all');
close(wbh_traj);
%% search finished,start adding results
result_list = zeros(sample_idx,2);
[~,cur_col] = find(f_cur == max(f_cur));
cur = cand_points_cur(cur_col,:);
for s_idx = sample_idx:-1:2
    result_list(s_idx,:) = cur;
    par_loc = find(parent_table.sample_idx == s_idx && parent_table.candidate_idx == cur_col);
    cur = cell2mat(parent_table.parent(par_loc));
    cur_col = parent_table.parent_idx(par_loc);
end
result_list(1,:) = cur;