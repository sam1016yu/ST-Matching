load('match_result.mat');
load('road&cell.mat');
%%
trajactory_tags = unique(trajactory.Tag...
    (trajactory.MatchedLat~=0&trajactory.MatchedLon~=0));
%%
close all;
for idx = 1:10
    figure;hold on;
    trajac = trajactory(trajactory.Tag == trajactory_tags(idx),:);
    cand_edges = unique(reshape(cell2mat(trajac.CandidateEdges),[],1));
    cand_roads = road_network(ismember(road_network.EdgeID,cand_edges),:);
    plot([cand_roads.Node1Lon cand_roads.Node2Lon],...
        [cand_roads.Node1Lat,cand_roads.Node2Lat],'k--'...
        ,'DisplayName','candidate edges');
    plot(trajac.Longitude,trajac.Latitude,'b.','DisplayName','raw points');
    plot(trajac.MatchedLon,trajac.MatchedLat,'rx',...
        'DisplayName','matched points');
    legend;
    hold off;
end
