function [trajactory] = mapCandidate(trajactory,road_network,road_cells,search_radius,cell_size,grid_size)
% find all candidate points
trajactory.CandidatePoints = cell(height(trajactory),1);
trajactory.CandidateEdges = cell(height(trajactory),1);
for point_idx = 1:height(trajactory)
    fprintf('Mapping candidates %i of %i \n',point_idx,height(trajactory));
    lon = trajactory.Longitude(point_idx);
    lat = trajactory.Latitude(point_idx);
    [road_ids, candPoints] = getCandidatePoints(lon,lat,...
        road_network,road_cells,search_radius,cell_size,grid_size);
    [row_can,~] = size(candPoints);
    trajactory.CandidatePoints(point_idx) = mat2cell(candPoints,row_can);
    trajactory.CandidateEdges(point_idx) = mat2cell(road_ids,row_can);
end
end

