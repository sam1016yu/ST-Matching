function [G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_network,grid_size,cut_step)
% for a given trajactory, find the area to search for shortest path.
% need to cut the grid so that only a portion of graph is used to search

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

%% convinence function for converting index
cell2rowcol = @(cellid) [ceil(cellid/grid_cols), ...
    mod(cellid,grid_cols)+(mod(cellid,grid_cols)==0)*grid_cols];
rowcol2cell = @(row,col) grid_cols*(row-1) + col; 

road_ids = containers.Map('KeyType','double','ValueType','double');
% start adding:
top_left_rowcol = cell2rowcol(top_left_id); bottom_right_rowcol = cell2rowcol(bottom_right_id);

% add road in range to cell
for col = (top_left_rowcol(2) - cut_step) : (bottom_right_rowcol(2) + cut_step)
    for row = (top_left_rowcol(1) - cut_step) : (bottom_right_rowcol(1) + cut_step)
        if  row>0 && row<=grid_rows && col>0 && col <= grid_cols
            if ~isempty(road_cells.roadID{rowcol2cell(row,col)})
                roads_to_add = road_cells.roadID{rowcol2cell(row,col)};
                for road_idx = 1:length(roads_to_add)
                    road_id = roads_to_add(road_idx);
                    road_ids(road_id) = 1;
                end
            end
        end
    end
end
road_ids = cell2mat(keys(road_ids));
search_edges = road_network(ismember(road_network(:,1),road_ids),:);
node_map = containers.Map('KeyType','double','ValueType','double');
[rows_search_edges,~] = size(search_edges);
node_num = 1;s = zeros(1,rows_search_edges); t = zeros(1,rows_search_edges);
%% building relationship between actual nodeID and the node ID in graph
warning('off','all');
for edge_idx = 1 : rows_search_edges
    if ~isKey(node_map,search_edges(edge_idx,2))
        node_map(search_edges(edge_idx,2)) = node_num;
        node1Graph = node_num;
        node_num = node_num + 1;
    else
        node1Graph = node_map(search_edges(edge_idx,2));
    end
    if ~isKey(node_map,search_edges(edge_idx,3))
        node_map(search_edges(edge_idx,3)) = node_num;
        node2Graph = node_num;
        node_num = node_num + 1;
    else
        node2Graph = node_map(search_edges(edge_idx,3));
    end
    s(edge_idx) = node1Graph;t(edge_idx) = node2Graph;
end
weights = deg2km(distance([search_edges(:,5),search_edges(:,4)],[search_edges(:,7),search_edges(:,6)]));
%% buding graph
G = graph(s,t,weights);
nodeID = cell2mat(keys(node_map)); nodeGraph = cell2mat(values(node_map));
node_table = table(nodeID,nodeGraph);
% save search_edges.mat search_edges
end