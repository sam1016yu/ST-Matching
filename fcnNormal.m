function Np = fcnNormal(ptn_cand,ptn_raw)
%fcnNormal: calculation of observation probablity of a given candidate
%point
% Input: ptn_cand: candidate points [lon lat]
%        ptn_raw:  GPS point[lon lat]
%%
mu = 0; sigma = 20/1000;
dist = distance(fliplr(ptn_cand),fliplr(ptn_raw),almanac('earth', 'wgs84'));
Np = exp(-(dist-mu)^2/(2*sigma^2))/(sqrt(2*pi)*sigma);
end
