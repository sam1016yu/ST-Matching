load('match_result.mat');
load('road&cell.mat');
%%
close all;
for idx = 1:10
    trajac = matched_trajactory(matched_trajactory.trajactory_tags == idx,:);
    if isempty(cell2mat(trajac.matched_points))
        continue
    end
    figure;hold on;
    matched_points = cell2mat(trajac.matched_points);
    raw_points = cell2mat(trajac.raw_points);
    cand_edges = unique(reshape(cell2mat(trajac.edges),[],1));
    cand_roads = road_network(ismember(road_network.EdgeID,cand_edges),:);
    plot([cand_roads.Node1Lon,cand_roads.Node2Lon]',...
        [cand_roads.Node1Lat,cand_roads.Node2Lat]','k-'...
        ,'DisplayName','candidate edges');
    plot(raw_points(:,1),raw_points(:,2),'b.','DisplayName','raw points');
    plot(matched_points(:,1),matched_points(:,2),'rx',...
        'DisplayName','matched points');
    hold off;
end
