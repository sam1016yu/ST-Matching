clear;clc;
%% cutting road network to grid
roadnetworkfilename = 'C:\Users\MiaoYu\Documents\我的坚果云\时空轨迹数据_FiveCities\BeijingCity\RoadNetwork_Beijing.txt';
cell_size = 0.1;
[road_network,road_cells,grid_size] = splitRoad2Cell(roadnetworkfilename, cell_size);
% load('road&cell.mat');grid_size = [747 924];
fprintf('Indexing grids done!\n');
%% splitting GPS trajactories
gpsfilename = 'C:\Users\MiaoYu\Documents\我的坚果云\时空轨迹数据_FiveCities\BeijingCity\GPS_Beijing.txt';
raw_gps_points = splitGPS2line(gpsfilename, 6, 5);
% load('GPS_Points.mat')
fprintf('Load GPS points done!\n');
%%
search_radius = 0.1;
trajactory = mapCandidate(raw_gps_points,road_network,road_cells,search_radius,cell_size,grid_size);
fprintf('Find candidate points done!\n');
%%
[G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_network,grid_size);
fprintf('Building graph done!\n');
%% fetching GPS candidate points
% single trajactories
trajactory_tags = unique(raw_gps_points.Tag);
for traj_idx = 1:length(trajactory_tags)
    fprintf('Mapping trajactory %i of %i \n',traj_idx,length(trajactory_tags));
    if trajactory_tags(traj_idx) == 0
        continue
    end
    traj_loc = trajactory.Tag == trajactory_tags(traj_idx);
    trajactory_to_match = trajactory(traj_loc,:);
    result = matchTrajactory(G,node_table,trajactory_to_match,road_network);
    trajactory.MatchedLon(traj_loc) = result(:,1);
    trajactory.MatchedLat(traj_loc) = result(:,2);
end