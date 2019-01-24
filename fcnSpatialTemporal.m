function F = fcnSpatialTemporal(ptn0_cand,ptn1_cand,ptn0_raw,ptn1_raw,delta_t,edg0_cand,edg1_cand,G,node_table,road_network)
%fcnSpatial:spatial analysis function
%   Implementation of equation 3 in the paper
%   Input:
%   ptn0_cand,ptn1_cand: two neighboring candidate points [lon lat]
%   ptn0_raw,ptn1_raw: GPS points of which candidate points belong
%   road_network: output of splitRoad2Cell
%%
[path_edges,dist_min] = findShortestPath(G,node_table,ptn0_cand,edg0_cand,ptn1_cand,edg1_cand,road_network);
Fs = fcnSpatial(ptn1_cand,ptn0_raw,ptn1_raw,dist_min);
Ft = fcnTemporal(delta_t,path_edges,dist_min,road_network);
F = Fs * Ft;
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
dist_actual = distance(fliplr(ptn0_raw),fliplr(ptn1_raw),almanac('earth', 'wgs84'));
Fs = Ncs*dist_actual/dist_shortest;
end

function Np = fcnNormal(ptn_cand,ptn_raw)
%fcnNormal: calculation of observation probablity of a given candidate
%point
% Input: ptn_cand: candidate points [lon lat]
%        ptn_raw:  GPS point[lon lat]
%%
mu = 0; sigma = 20/1000;
dist = distance(fliplr(ptn_cand),fliplr(ptn_raw),almanac('earth', 'wgs84'));
Np = exp((dist-mu)^2/(2*sigma^2))/(sqrt(2*pi)*sigma);
end

function Ft = fcnTemporal(delta_t,path_edges,dist_min,road_network)
    avg_speed = dist_min / delta_t * 60 * 60;
    avg_speed_vec = repmat(avg_speed,1,length(path_edges));
    speed_limit_vec = getSpeedLimits(road_network.Type(ismember(road_network.EdgeID,path_edges)));
    Ft = pdist([avg_speed_vec;speed_limit_vec],'cosine');
end