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
% vectors of candidate points ,in each row: [lon lat]
%% search for candidate edges within given grid
% locate center grid first
center_cell_id = find(grid.startLat<=lat & grid.endLat>= lat & ...
    grid.startLon <= lon & grid.endLon >= lon);
%% search surrounding grids
extra_grids_count = ceil(search_radius/cell_size)-1;
road_ids = searchSurroundingGrids(grid_size,center_cell_id,extra_grids_count,grid);
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

