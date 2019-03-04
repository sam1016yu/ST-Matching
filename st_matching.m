clear;clc;
%% cutting road network to grid
% roadnetworkfilename = 'RoadNetwork_Beijing.txt';
% cell_size = 0.1;
% [road_network,road_types,road_cells,road_ids,grid_size] = splitRoad2Cell(roadnetworkfilename, cell_size);
% save road&cell.mat road_cells road_network cell_size grid_size road_types
load('mat_road&cell.mat');grid_size = [747 924];
fprintf('Indexing grids done!\n');
%% splitting GPS trajactories
gpsfilename = 'GPS_Beijing.txt';
% raw_gps_points = splitGPS2line(gpsfilename, 6, 5);
load('mat_GPS_Points.mat')
fprintf('Load GPS points done!\n');
% searching for candidate points
% fetching GPS candidate points
trajactory_tags = unique(raw_gps_points(:,7));
matched_trajactory = table(trajactory_tags);
matched_trajactory.raw_points = cell(length(trajactory_tags),1);
matched_trajactory.matched_points = cell(length(trajactory_tags),1);
matched_trajactory.edges = cell(length(trajactory_tags),1);
fId = fopen('trajactory.log','w+');
warning('off','all')
for traj_idx = 1:100
% for traj_idx = 1:length(trajactory_tags)
    try
        fprintf(1,'Mapping trajactory %i of %i,Time: %s \n',traj_idx,length(trajactory_tags), datestr(now));
        fprintf(fId,'Mapping trajactory %i of %i,Time: %s \n',traj_idx,length(trajactory_tags), datestr(now));
        if trajactory_tags(traj_idx) == 0
            continue
        end
% traj_idx = 16; 
        traj_loc = raw_gps_points(:,7) == trajactory_tags(traj_idx);
        trajactory_to_match = raw_gps_points(traj_loc,:);
        [path_result,point_result] = matchTrajactory(trajactory_to_match,road_network,road_types,road_cells,road_ids,cell_size,grid_size);
        [rows_point,~] = size(point_result);
        matched_trajactory.raw_points(traj_idx) = mat2cell(trajactory_to_match(:,3:4),rows_point);
        matched_trajactory.matched_points(traj_idx) = mat2cell(point_result,rows_point);
        matched_trajactory.edges(traj_idx) = mat2cell(path_result',length(path_result));
    catch e 
        % for excepetion occured (e.g., only one point in a trajactory,
        % no route find in graph...), catch it here and print in in log
        fprintf(1,'Error:\n%s\n',e.message);
        fprintf(fId,'Error:\n%s\n',e.message);
        continue
    end
end
warning('on','all')
fclose(fId);
save match_result_all.mat matched_trajactory