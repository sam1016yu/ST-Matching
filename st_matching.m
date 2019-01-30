clear;clc;
%% cutting road network to grid
roadnetworkfilename = 'RoadNetwork_Beijing.txt';
cell_size = 0.1;
[road_network,road_cells,grid_size] = splitRoad2Cell(roadnetworkfilename, cell_size);
save road&cell.mat road_cells road_network cell_size grid_size
% load('road&cell.mat');grid_size = [747 924];
fprintf('Indexing grids done!\n');
%% splitting GPS trajactories
gpsfilename = 'GPS_Beijing.txt';
raw_gps_points = splitGPS2line(gpsfilename, 6, 5);
% load('GPS_Points.mat')
fprintf('Load GPS points done!\n');
% searching for candidate points
search_radius = 0.1;
trajactory = mapCandidate(raw_gps_points,road_network,road_cells,search_radius,cell_size,grid_size);
fprintf('Find candidate points done!\n');
%% cut a smaller portion and build graph.
% take a lot of time, do this only once for all trajactories to save time
[G,node_table] = cutGridforTrajactory(trajactory,road_cells,road_network,grid_size);
fprintf('Building graph done!\n');
% fetching GPS candidate points
trajactory_tags = unique(raw_gps_points.Tag);
fId = fopen('trajactory.log','w+');
for traj_idx = 1:length(trajactory_tags)
    try
        fprintf(1,'Mapping trajactory %i of %i,Time: %s \n',traj_idx,length(trajactory_tags), datestr(now));
        fprintf(fId,'Mapping trajactory %i of %i,Time: %s \n',traj_idx,length(trajactory_tags), datestr(now));
        if trajactory_tags(traj_idx) == 0
            continue
        end
        traj_loc = trajactory.Tag == trajactory_tags(traj_idx);
        trajactory_to_match = trajactory(traj_loc,:);
        result = matchTrajactory(G,node_table,trajactory_to_match,road_network);
        trajactory.MatchedLon(traj_loc) = result(:,1);
        trajactory.MatchedLat(traj_loc) = result(:,2);
    catch e 
        % for excepetion occured (e.g., only one point in a trajactory,
        % no route find in graph...), catch it here and print in in log
        fprintf(1,'Error:\n%s\n',e.message);
        fprintf(fId,'Error:\n%s\n',e.message);
        continue
    end

end
fclose(fId);
save match_result.mat trajactory