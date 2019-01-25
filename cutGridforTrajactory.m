function [G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_network,grid_size,cut_step)
% for a given trajactory, find the area to search for shortest path.
% need to cut the grid so that only a portion of graph is used to search


if nargin < 5
    cut_step = ceil(min(grid_size) / 10);  % default cut size
end
%% convinence function for finding
findCellByLonLat = @(lon,lat) find(road_cells.startLat<=lat & road_cells.endLat>= lat & ...
    road_cells.startLon <= lon & road_cells.endLon >= lon);

traj_min_lon = min(trajactory.Longitude); traj_max_lon = max(trajactory.Longitude);
traj_min_lat = min(trajactory.Latitude); traj_max_lat = max(trajactory.Latitude);

% corner of grid, two diag items
top_left_id = findCellByLonLat(traj_min_lon,traj_min_lat);
bottom_right_id = findCellByLonLat(traj_max_lon,traj_max_lat);
%% add all edges in range
% function that convert between cellId and  [row_id,col_id]
% convenient for searching surrounding grids
grid_cols = grid_size(2);grid_rows = grid_size(1);

cell2rowcol = @(cellid) [ceil(cellid/grid_cols), ...
    mod(cellid,grid_cols)+(mod(cellid,grid_cols)==0)*grid_cols];
rowcol2cell = @(row,col) grid_cols*(row-1) + col; 
isValidRowCol = @(row,col) row>0 && row<=grid_rows && col>0 && col <= grid_cols;

road_ids = [];
% start adding:
top_left_rowcol = cell2rowcol(top_left_id); bottom_right_rowcol = cell2rowcol(bottom_right_id);
for col = (top_left_rowcol(2) - cut_step) : (bottom_right_rowcol(2) + cut_step)
    for row = (top_left_rowcol(1) - cut_step) : (bottom_right_rowcol(1) + cut_step)
        if isValidRowCol(row,col)
            road_ids = [road_ids cell2mat(road_cells(rowcol2cell(row,col),:).roadID)];
        end
    end
end
road_ids = unique(road_ids);
search_edges = road_network(ismember(road_network.EdgeID,road_ids),:);
node_table = table; node_table.nodeID = zeros(0);node_table.nodeGraph = zeros(0);
node_num = 1;s = zeros(1,height(search_edges)); t = zeros(1,height(search_edges));
weights = zeros(1,height(search_edges));
%%
warning('off','all');
for edge_idx = 1 : height(search_edges)
    edge = search_edges(edge_idx,:);
    if ~ismember(edge.Node1ID,node_table.nodeID)
        node_table.nodeID(node_num) = edge.Node1ID;node_table.nodeGraph(node_num) = node_num;
        node1Graph = node_num;
        node_num = node_num + 1;
    else
        node1Graph = node_table.nodeGraph(node_table.nodeID == edge.Node1ID);
    end
    if ~ismember(edge.Node2ID,node_table.nodeID)
        node_table.nodeID(node_num) = edge.Node2ID;node_table.nodeGraph(node_num) = node_num;
        node2Graph = node_num;
        node_num = node_num + 1;
    else
        node2Graph = node_table.nodeGraph(node_table.nodeID == edge.Node2ID);
    end
    s(edge_idx) = node1Graph;t(edge_idx) = node2Graph;
    weights(edge_idx) = distance(edge.Node1Lat,edge.Node1Lon,edge.Node2Lat,edge.Node2Lon,almanac('earth', 'wgs84'));
end
%%
G = graph(s,t,weights);
end