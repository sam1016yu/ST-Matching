function Np = fcnNormal(ptn_cand,ptn_raw)
%fcnNormal: calculation of observation probablity of a given candidate
%point
% Input: ptn_cand: candidate points [lon lat]
%        ptn_raw:  GPS point[lon lat]
%%
sigma = 20;
dist = deg2km(distance(fliplr(ptn_cand),fliplr(ptn_raw)))*1000;
Np = 5*exp(-dist^2/(2*sigma^2))/(sqrt(2*pi)*sigma);
end
