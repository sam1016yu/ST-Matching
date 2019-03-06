function [F,path_edges] = fcnSpatialTemporal(ptn0_cand,ptn1_cand,ptn0_raw,...
    ptn1_raw,delta_t,edg0_cand,edg1_cand,G,node_table,road_network,speed_limits)
%fcnSpatial:spatial analysis function
%   Implementation of equation 3 in the paper
%   Input:
%   ptn0_cand,ptn1_cand: two neighboring candidate points [lon lat]
%   ptn0_raw,ptn1_raw: GPS points of which candidate points belong
%   road_network: output of splitRoad2Cell
%%
% try
    [path_edges,dist_min] = findShortestPath(G,node_table,ptn0_cand,...
        edg0_cand,ptn1_cand,edg1_cand,road_network);
    if dist_min == 0
%         dist_min = 2*deg2km(distance(fliplr(ptn0_cand),fliplr(ptn1_cand)));
        F = 0;
    elseif dist_min == Inf
        F = -99999;
    else
        Fs = fcnSpatial(ptn1_cand,ptn0_raw,ptn1_raw,dist_min);
        Ft = fcnTemporal(delta_t,path_edges,dist_min,speed_limits,road_network);
        F = Fs * Ft;
    end
% catch ME
%     if strcmp(ME.identifier,'shortestPath:noPath')
%         F = 0; path_edges = [];
%     else
%         rethrow(ME)
%     end
% end
end

function Fs = fcnSpatial(ptn1_cand,ptn0_raw,ptn1_raw,dist_shortest)
%fcnSpatial:spatial analysis function
%   Implementation of equation 3 in the paper
%   Input:
%   ptn0_cand,ptn1_cand: two neighboring candidate points [lon lat]
%   ptn0_raw,ptn1_raw: GPS points of which candidate points belong
%   road_network: output of splitRoad2Cell
%%
Ncs = fcnNormal(ptn1_cand,ptn1_raw);
% distance between two GPS points
dist_ratio = deg2km(distance(fliplr(ptn0_raw),fliplr(ptn1_raw)))/dist_shortest;
if dist_ratio >= 1.5
    dist_ratio = 0;
%     dist_ratio = 1 + log10(dist_ratio)/4;
end
Fs = Ncs*dist_ratio;
end


function Ft = fcnTemporal(delta_t,path_edges,dist_min,speed_limits,road_network)
    avg_speed = dist_min / delta_t * 60 * 60;
    path_edges = unique(path_edges);
    avg_speed_vec = repmat(avg_speed,1,length(path_edges));
    speed_limit_vec = speed_limits(ismember(road_network(:,1),path_edges),:)';
    % cosine distance of two speed vecotrs
    Ft = sum(avg_speed_vec.*speed_limit_vec)/...
        sqrt(sum(avg_speed_vec.^2)*sum(speed_limit_vec.^2));
end