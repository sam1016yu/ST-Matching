function [path_list,point_list] = matchTrajactory(trajactory,...
    road_network,road_types,road_cells,road_ids_all,cell_size,grid_size)
[rows_trajactory,~] = size(trajactory);
assert(rows_trajactory>1,'A trajactory need to be more than one point!');
search_radius = 0.1;
% load('search_edges.mat');close all;figure;hold on; 
% plot([search_edges(:,4),search_edges(:,6)]',[search_edges(:,5),search_edges(:,7)]','k-');
% axis equal;
cand_points_edges = mapCandidate(trajactory,road_network,road_cells,road_ids_all,search_radius,cell_size,grid_size);
[G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_ids_all,road_network,grid_size,30);
% fprintf('Building graph done!\n');
% Find matched sequence
% see 'findMatchedSquence.png' for detail
%% first round
% cand_points_prev = cell2mat(trajactory.CandidatePoints(1));
% cand_edges_prev = cell2mat(trajactory.CandidateEdges(1));
cand_points_prev = cand_points_edges(:,1:2,1);
cand_edges_prev = cand_points_edges(:,3,1);
sample_point_prev = [trajactory(1,3) trajactory(1,4)];
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
for sample_idx = 2:rows_trajactory
%     fprintf('sample idx:%i \n',sample_idx);
%     f_prev
    delta_t = trajactory(sample_idx,2) - trajactory(sample_idx-1,2);
    sample_point_cur = [trajactory(sample_idx,3),...
        trajactory(sample_idx,4)];
    cand_points_cur = cand_points_edges(:,1:2,sample_idx);
    cand_edges_cur =  cand_points_edges(:,3,sample_idx);
    [cand_points_cur_row,~] = size(cand_points_cur);
    f_cur = zeros(1,cand_points_cur_row); 
    for idx_cur = 1 : length(f_cur)
        cur_max = -Inf;
        for idx_prev = 1: length(f_prev)
            [new_score,path_edges] = fcnSpatialTemporal(cand_points_prev(idx_prev,:),...
                cand_points_cur(idx_cur,:),sample_point_prev,sample_point_cur,...
                delta_t,cand_edges_prev(idx_prev),cand_edges_cur(idx_cur),...
                G,node_table,road_network,road_types);
            alt = f_prev(idx_prev) + new_score;
            if alt > cur_max
%                  fprintf('sample:%i,idx_cur:%i,idx_prev:%i,',sample_idx,idx_cur,idx_prev);
%                 fprintf('current best!\n');
%                 fprintf('score:%0.8f\n',new_score);
                cur_max = alt;
                parent_loc = find(parent_table.sample_idx==sample_idx & parent_table.candidate_idx == idx_cur);
                if ~any(parent_loc)
                    parent_loc = height(parent_table)+1;
                end
%                 fcnSpatialTemporal(cand_points_prev(idx_prev,:),...
%                 cand_points_cur(idx_cur,:),sample_point_prev,sample_point_cur,...
%                 delta_t,cand_edges_prev(idx_prev),cand_edges_cur(idx_cur),...
%                 G,node_table,road_network,road_types);
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
%     [cand_points_prev,cand_edges_prev,f_prev] = cutWindow(cand_points_prev,cand_edges_prev,f_prev);
end
% fprintf('done searching,final score:\n');
% f_prev
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

function [cand_points_edges] = mapCandidate(trajactory,road_network,...
    road_cells,road_ids_all,search_radius,cell_size,grid_size)
% find all candidate points
top_k = 5;
[rows_trajactory,~] = size(trajactory);
cand_points_edges = zeros(top_k,3,rows_trajactory);
for point_idx = 1:rows_trajactory
%     fprintf('Mapping candidates %i of %i \n',point_idx,height(trajactory));
    lon = trajactory(point_idx,3);
    lat = trajactory(point_idx,4);
    [road_ids, candPoints] = getCandidatePoints(lon,lat,...
        road_network,road_cells,road_ids_all,search_radius,cell_size,grid_size,top_k);
    cand_points_edges(:,1:2,point_idx) = candPoints;
    cand_points_edges(:,3,point_idx) = road_ids;
end
end

% function [cand_points,cand_edges,f] = cutWindow(cand_points_prev,cand_edges_prev,f_prev)
%     window_size = 5;
%     [cand_points_prev_row,~] = size(cand_points_prev);
%     if cand_points_prev_row <= window_size
%         cand_points = cand_points_prev;cand_edges = cand_edges_prev;f = f_prev;
%     else
%         matrix_all = [f_prev',cand_edges_prev,cand_points_prev];
%         matrix_all = sortrows(matrix_all,1,'descend');
%         f = matrix_all(1:window_size,1)';cand_points = matrix_all(1:window_size,3:4);
%         cand_edges = matrix_all(1:window_size,2);
%     end
% end