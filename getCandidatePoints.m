function [candidate_points] = getCandidatePoints(lon,lat,road,grid,search_radius,cell_size, grid_size)
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
% vectors of longitude and latitude of candidate points
%% search for candidate edges within given grid
% locate center grid first
center_cell_id = find(grid.startLat<=lat & grid.endLat>= lat & ...
    grid.startLon <= lon & grid.endLon >= lon);
% initialize road ids with edges in the center grid
road_ids = cell2mat(grid(center_cell_id,:).roadID);
% consider looking around center grid based on the given search radius and
% grid size used previously
grid_rows = grid_size(1);grid_cols = grid_size(2);
% function that convert between cellId and  [row_id,col_id]
% convenient for searching surrounding grids
cell2rowcol = @(cellid) [ceil(cellid/grid_cols), ...
    mod(cellid,grid_cols)+(mod(cellid,grid_cols)==0)*grid_cols];
rowcol2cell = @(row,col) grid_cols*(row-1) + col; 
isValidRowCol = @(row,col) row>0 && row<=grid_rows && col>0 && col <= grid_cols;
%% search surrounding grids
center_rowcol = cell2rowcol(center_cell_id);
center_row = center_rowcol(1); center_col = center_rowcol(2);
extra_grids_count = ceil(search_radius/cell_size)-1;
if extra_grids_count > 0
    for step_col = -extra_grids_count:extra_grids_count
        for step_row = -extra_grids_count:extra_grids_count
            if isValidRowCol(center_row+step_row,center_col+step_col)
                road_ids = [road_ids cell2mat(grid(rowcol2cell(center_row+step_row,center_col+step_col),:).roadID)];
            end
        end
    end
    road_ids = unique(road_ids);
end
%% project point to candidate edges to find candidate points
candidate_edges = road(ismember(road.EdgeID,road_ids),:);
candidate_points = zeros(height(candidate_edges),2);
for edge_idx = 1 : height(candidate_edges)
    candidate_points(edge_idx,:) = project2Line(lon,lat,candidate_edges(edge_idx,:));
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
    candidate = r*ab - a;
end
end

