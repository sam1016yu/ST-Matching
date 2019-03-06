load('match_result_all.mat');
load('mat_road&cell.mat');
%%
close all;
% idx_to_plot = [46];
for idxx = 1:100
%     idx = 3;
%     subplot(3,5,idxx);
%     idx = idx_to_plot(idxx);
% idx = 46;
    idx = idxx;
    trajac = matched_trajactory(matched_trajactory.trajactory_tags == idx,:);
    if isempty(cell2mat(trajac.matched_points))
        continue
    end
    fig = figure;
    fig.Visible = 'off';
    hold on;
    matched_points = cell2mat(trajac.matched_points);
    raw_points = cell2mat(trajac.raw_points);
    cand_edges = unique(reshape(cell2mat(trajac.edges),[],1));
    cand_roads = road_network(ismember(road_network(:,1),cand_edges),:);
    plot([cand_roads(:,4),cand_roads(:,6)]',...
        [cand_roads(:,5),cand_roads(:,7)]','k-'...
        ,'DisplayName','candidate edges');
    plot(raw_points(:,1),raw_points(:,2),'b.','DisplayName','raw points');
    plot(matched_points(:,1),matched_points(:,2),'rx',...
        'DisplayName','matched points');
    title(num2str(idx));axis equal;
    if exist('result')~=7
        mkdir result
    end 
    print(fig,fullfile(pwd,'result',sprintf('traj_%i.png',idx)),'-dpng')
%     print(fig,strcat('result_new/traj_',mat2str(idx)),'-dpng');
    hold off;
    close(fig);
end
