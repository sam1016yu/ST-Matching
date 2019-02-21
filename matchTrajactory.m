function [path_list,point_list] = matchTrajactory(gps_points,road_network,road_cells,cell_size,grid_size)

assert(height(gps_points)>1,'A trajactory need to be more than one point!');
search_radius = 0.1;
trajactory = mapCandidate(gps_points,road_network,road_cells,search_radius,cell_size,grid_size);
[G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_network,grid_size,30);
fprintf('Building graph done!\n');
% Find matched sequence
% see 'findMatchedSquence.png' for detail
%% first round
cand_points_prev = cell2mat(trajactory.CandidatePoints(1));
cand_edges_prev = cell2mat(trajactory.CandidateEdges(1));
sample_point_prev = [trajactory.Longitude(1) trajactory.Latitude(1)];
[cand_points_prev_row,~] = size(cand_points_prev);
f_prev = zeros(1,cand_points_prev_row);
for cand_idx = 1 : cand_points_prev_row
    % initialize first round scores with fcnNormal
    f_prev(cand_idx) = fcnNormal(cand_points_prev(cand_idx,:),sample_point_prev);
end
%% second round search
parent_table = table;
parent_table.sample_idx = zeros(0);parent_table.candidate_idx = zeros(0);
parent_table.parent = cell(0);parent_table.parent_idx = zeros(0);
parent_table.path = cell(0);
for sample_idx = 2:height(trajactory)
    delta_t = trajactory.Timestamp(sample_idx) - trajactory.Timestamp(sample_idx-1);
    sample_point_cur = [trajactory.Longitude(sample_idx),...
        trajactory.Latitude(sample_idx)];
    cand_points_cur = cell2mat(trajactory.CandidatePoints(sample_idx));
    cand_edges_cur = cell2mat(trajactory.CandidateEdges(sample_idx));
    [cand_points_cur_row,~] = size(cand_points_cur);
    f_cur = zeros(1,cand_points_cur_row); 
    for idx_cur = 1 : length(f_cur)
        cur_max = -Inf;
        for idx_prev = 1: length(f_prev)
            [new_score,path_edges] = fcnSpatialTemporal(cand_points_prev(idx_prev,:),...
                cand_points_cur(idx_cur,:),sample_point_prev,sample_point_cur,...
                delta_t,cand_edges_prev(idx_prev),cand_edges_cur(idx_cur),...
                G,node_table,road_network);
            alt = f_prev(idx_prev) + new_score;
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
                parent_table.path(parent_loc) = mat2cell(path_edges,1);
            end
            f_cur(idx_cur) = cur_max; 
        end
    end
    cand_points_prev = cand_points_cur;cand_edges_prev = cand_edges_cur;
    sample_point_prev = sample_point_cur;f_prev = f_cur;
end
%% search finished,start adding results
point_list = zeros(sample_idx,2);path_list = [];
[~,cur_col] = find(f_cur == max(f_cur));
cur_col = cur_col(1);
cur = cand_points_cur(cur_col,:);
for s_idx = sample_idx:-1:2
    point_list(s_idx,:) = cur;
    path_list = [path_list parent_table.path{parent_table.sample_idx == s_idx & parent_table.candidate_idx == cur_col}];
    par_loc = find(parent_table.sample_idx == s_idx & parent_table.candidate_idx == cur_col);
    cur = cell2mat(parent_table.parent(par_loc));
    cur_col = parent_table.parent_idx(par_loc);
end
point_list(1,:) = cur;
end

function [trajactory] = mapCandidate(trajactory,road_network,road_cells,search_radius,cell_size,grid_size)
% find all candidate points
trajactory.CandidatePoints = cell(height(trajactory),1);
trajactory.CandidateEdges = cell(height(trajactory),1);
for point_idx = 1:height(trajactory)
%     fprintf('Mapping candidates %i of %i \n',point_idx,height(trajactory));
    lon = trajactory.Longitude(point_idx);
    lat = trajactory.Latitude(point_idx);
    [road_ids, candPoints] = getCandidatePoints(lon,lat,...
        road_network,road_cells,search_radius,cell_size,grid_size);
    [row_can,~] = size(candPoints);
    trajactory.CandidatePoints(point_idx) = mat2cell(candPoints,row_can);
    trajactory.CandidateEdges(point_idx) = mat2cell(road_ids,row_can);
end
end

