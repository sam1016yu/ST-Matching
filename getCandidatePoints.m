function [road_ids,candidate_points] = getCandidatePoints(lon,lat,road_network,road_cells,search_radius,cell_size, grid_size,top_k)
%find candidate points within radius r
% Parameters:
% lon,lat: longitude and latitude of the point to be searched
% raod: road network information. (outputs from splitRoad2Cell)
% grid: grid info (outputs from splitRoad2Cell)
% search_radius: the radius(km) within which candidate points are selected
% cell size: size of cutted cell in grid
% grid size: m*n array
%
% Output:
% vectors of candidate points ,in each row: [lon lat]

if nargin < 8
    top_k = 5;
end
%% search for candidate edges within given grid
% locate center grid first
center_cell_id = find(road_cells.startLat<=lat & road_cells.endLat>= lat & ...
    road_cells.startLon <= lon & road_cells.endLon >= lon);
%% search surrounding grids
extra_grids_count = ceil(search_radius/cell_size)-1;
road_ids = searchSurroundingGrids(grid_size,center_cell_id,extra_grids_count,road_cells);
%% project point to candidate edges to find candidate points
candidate_edges = road_network(ismember(road_network.EdgeID,road_ids),:);
candidate_points = zeros(height(candidate_edges),2);
for edge_idx = 1 : height(candidate_edges)
    candidate_points(edge_idx,:) = project2Line(lon,lat,candidate_edges(edge_idx,:));
end
[row_p,~] = size(candidate_points);
if row_p > top_k
    points = mat2cell(candidate_points,ones(1,row_p)); res_table = table(points,road_ids);
    res_table.distance = distance(fliplr(cell2mat(res_table.points)),repmat([lat lon],row_p,[]));
    res_table.distance = deg2km(res_table.distance);
    res_table = sortrows(res_table,{'distance'},{'ascend'});
    candidate_points = cell2mat(res_table.points(1:top_k));
    road_ids = res_table.road_ids(1:top_k);
end
end

function [candidate] = project2Line(lon,lat,edge)
% see detail in:
% http://blog.sina.com.cn/s/blog_5d5c80840101bnhw.html
% http://s3.sinaimg.cn/orignal/5d5c8084gd638da294362&690
a = [edge.Node1Lon edge.Node1Lat];
b = [edge.Node2Lon edge.Node2Lat];
p = [lon lat];
ap = p-a; ab = b-a; r = sum(ap.*ab)/sum(ab.^2);
if r <= 0
    candidate = a;
elseif r>=1
    candidate = b;
else
    candidate = r*ab + a;
end
end

function [road_ids] = searchSurroundingGrids(grid_size,center_cell_id,search_steps,grid)
%searchSurroundingGrids
%   search surrounding edges for candidate edges
%   Input:
%   grid_size: m*n array
%   search_steps: steps that search is conducted
%   grid: grided road network info

% consider looking around center grid based on the given search radius and
% grid size used previously
grid_cols = grid_size(2);grid_rows = grid_size(1);

% function that convert between cellId and  [row_id,col_id]
% convenient for searching surrounding grids
cell2rowcol = @(cellid) [ceil(cellid/grid_cols), ...
    mod(cellid,grid_cols)+(mod(cellid,grid_cols)==0)*grid_cols];
rowcol2cell = @(row,col) grid_cols*(row-1) + col;
isValidRowCol = @(row,col) row>0 && row<=grid_rows && col>0 && col <= grid_cols;

road_ids = cell2mat(grid(center_cell_id,:).roadID)';
center_rowcol = cell2rowcol(center_cell_id);
center_row = center_rowcol(1); center_col = center_rowcol(2);
if search_steps > 0
    for step_col = -search_steps:search_steps
        for step_row = -search_steps:search_steps
            if isValidRowCol(center_row+step_row,center_col+step_col)
                road_ids = [road_ids cell2mat(grid(rowcol2cell(center_row+step_row,center_col+step_col),:).roadID)];
            end
        end
    end
    road_ids = unique(road_ids)';
elseif any(road_ids)
    road_ids = unique(road_ids);
else
    search_steps = 1;
    while ~any(road_ids)
        for step_col = -search_steps:search_steps
            for step_row = -search_steps:search_steps
                if isValidRowCol(center_row+step_row,center_col+step_col)
                    road_ids = [road_ids cell2mat(grid(rowcol2cell(center_row+step_row,center_col+step_col),:).roadID)];
                end
            end
        end
        road_ids = unique(road_ids)';
        search_steps = search_steps + 1;
    end
end
end



