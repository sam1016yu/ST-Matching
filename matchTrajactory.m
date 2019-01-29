function [result_list] = matchTrajactory(G,node_table,trajactory,road_network)

%% Find matched sequence start
%first round
assert(height(trajactory)>1,'A trajactory need to be more than one point!');
cand_points_prev = cell2mat(trajactory.CandidatePoints(1));
cand_edges_prev = cell2mat(trajactory.CandidateEdges(1));
sample_point_prev = [trajactory.Longitude(1) trajactory.Latitude(1)];
[cand_points_prev_row,~] = size(cand_points_prev);
f_prev = zeros(1,cand_points_prev_row);
for cand_idx = 1 : cand_points_prev_row
    f_prev(cand_idx) = fcnNormal(cand_points_prev(cand_idx,:),sample_point_prev);
end
%% second round search
parent_table = table;
parent_table.sample_idx = zeros(0);parent_table.candidate_idx = zeros(0);
parent_table.parent = cell(0);parent_table.parent_idx = zeros(0);
for sample_idx = 2:height(trajactory)
    delta_t = trajactory.Timestamp(sample_idx) - trajactory.Timestamp(sample_idx-1);
    sample_point_cur = [trajactory.Longitude(sample_idx),...
        trajactory.Latitude(sample_idx)];
    cand_points_cur = cell2mat(trajactory.CandidatePoints(sample_idx));
    cand_edges_cur = cell2mat(trajactory.CandidateEdges(sample_idx));
    [cand_points_cur_row,~] = size(cand_points_cur);
    f_cur = zeros(1,cand_points_cur_row); 
%     wbh_calc = waitbar(0,'Caluculating Scores for this edge');
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
%                 waitbar((idx_prev+(idx_cur-1)*length(f_prev))/(length(f_prev)*length(f_cur)),wbh_calc);
            end
            f_cur(idx_cur) = cur_max; 
        end
    end
%     close(wbh_calc);
    cand_points_prev = cand_points_cur;cand_edges_prev = cand_edges_cur;
    sample_point_prev = sample_point_cur;f_prev = f_cur;
end
warning('on','all');
%% search finished,start adding results
result_list = zeros(sample_idx,2);
[~,cur_col] = find(f_cur == max(f_cur));
cur_col = cur_col(1);
cur = cand_points_cur(cur_col,:);
for s_idx = sample_idx:-1:2
    result_list(s_idx,:) = cur;
    par_loc = find(parent_table.sample_idx == s_idx & parent_table.candidate_idx == cur_col);
    cur = cell2mat(parent_table.parent(par_loc));
    cur_col = parent_table.parent_idx(par_loc);
end
result_list(1,:) = cur;
end

