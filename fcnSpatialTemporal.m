function Fs = fcnSpatialTemporal(ptn0_cand,ptn1_cand,ptn0_raw,ptn1_raw,road_network)
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


end

function Fs = fcnSpatial(ptn0_cand,ptn1_cand,ptn0_raw,ptn1_raw,road_network)
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
