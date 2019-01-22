function road_cells = splitRoad2Cell(filename,grid_size)
% filename: the raw txt file of road network, in required format
% grid size: the desired grid size (in km)
%% read raw text file of road network
delimiter = ',';
formatSpec = '%f%f%f%f%f%f%f%C%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN,  'ReturnOnError', false);
fclose(fileID);
road_network_raw = table(dataArray{1:end-1}, 'VariableNames', {'EdgeID','Node1ID','Node2ID','Node1Lon','Node1Lat','Node2Lon','Node2Lat','Type'});
clearvars filename delimiter formatSpec fileID dataArray ans;
%% cut grids
% determining 4 points on the edges
lon_max = max(max(road_network_raw.Node1Lon),max(road_network_raw.Node2Lon));
lon_min = min(min(road_network_raw.Node1Lon),min(road_network_raw.Node2Lon));
lat_max = max(max(road_network_raw.Node1Lat),max(road_network_raw.Node2Lat));
lat_min = min(min(road_network_raw.Node1Lat),min(road_network_raw.Node2Lat));
% calculate area size
area_height = max(distance([lat_max,lon_min],[lat_min,lon_min]),distance([lat_max,lon_max],[lat_min,lon_max]));
area_width = max(distance([lat_max,lon_max],[lat_max,lon_min]),distance([lat_min,lon_max],[lat_min,lon_min]));
area_height = deg2km(area_height);area_width = deg2km(area_width);
% calculate number of grids
y_size = ceil(area_height/grid_size);x_size = ceil(area_width/grid_size);
lon_vec = linspace(lon_min,lon_max,x_size); lat_vec = linspace(lat_min,lat_max,y_size);
% [lon_grid,lat_grid] = meshgrid(lon_vec,lat_vec);
%% find grid 
roadID = findEdgeIndex(lon_vec,lat_vec,road_network_raw);
road_cells = table(roadID);
%% fill info of each cell
road_cells.cellID = (1:height(road_cells))';
road_cells.startLon = repmat(lon_vec(1:end-1),1,y_size-1)';
road_cells.endLon = repmat(lon_vec(2:end),1,y_size-1)';
road_cells.startLat = reshape(repmat(lat_vec(1:end-1),x_size-1,1),[],1);
road_cells.endLat = reshape(repmat(lat_vec(2:end),x_size-1,1),[],1);
road_cells = [road_cells(:,2:end) road_cells(:,1)];
% clearvars -except road_cells
end


function road_cells = findEdgeIndex(lon_vec,lat_vec,road_network_raw)
%% given gidded network and edge,find whcih grid the edge belong to
wbh = waitbar(0,'indexing edges...');
road_cells = cell((length(lon_vec)-1) * (length(lat_vec)-1),1);
for edges_idx = 1: height(road_network_raw)
    edge_index = [];
    edges = road_network_raw(edges_idx,:);
    [test_lon_vec,test_lat_vec] = divideEdges(edges,lon_vec(2)-lon_vec(1),lat_vec(2)-lat_vec(1));
    for test_idx = 1:length(test_lon_vec)
        test_lon = test_lon_vec(test_idx);test_lat = test_lat_vec(test_idx);
        [~,col_lon,~] = find(lon_vec>=test_lon);[~,col_lat,~] = find(lat_vec>=test_lat);
        lon_idx = col_lon(1) -1;lat_idx = col_lat(1) -1;
        if lon_idx == 0
            lon_idx = 1;
        elseif lat_idx == 0
            lat_idx = 1;
        end
        idx_lon_lat = lon_idx + (length(lon_vec)-1)*(lat_idx -1 );
        assert(idx_lon_lat > 0);
        edge_index = [edge_index idx_lon_lat];
    end
    edge_index = unique(edge_index);
    for idx = 1:length(edge_index)
        grid_id = edge_index(idx);
        if isempty(road_cells{grid_id,1})
            road_cells{grid_id,1} = edges.EdgeID;
        else
            road_cells{grid_id,1} = [road_cells{grid_id,1} edges.EdgeID];
        end
    end
    waitbar(edges_idx/height(road_network_raw),wbh);
end
close(wbh);
end


function [test_lon,test_lat] = divideEdges(edges,lon_grid,lat_grid)
%%
    % calculate coordiinates to be tested
    % e.g. if edge is divided into 3 portions,
    % test start, end, 1/3, 2/3 division point
    
    % search direction
    start_lon = min(edges.Node1Lon,edges.Node2Lon);end_lon = max(edges.Node1Lon,edges.Node2Lon);
    start_lat = min(edges.Node1Lat,edges.Node2Lat);end_lat = max(edges.Node1Lat,edges.Node2Lat);
    % span of the search
    lon_span = ceil((end_lon - start_lon)/(lon_grid));
    lat_span = ceil((end_lat - start_lat)/(lat_grid));
    span = max(lon_span,lat_span);
    % definite proportional division point formula 定比分点公式
    porDiv = @(lambda,start_cor,end_cor) (start_cor+lambda*end_cor)/(1+lambda);
    if span == 1
        test_lon = [edges.Node1Lon,edges.Node2Lon];
        test_lat = [edges.Node1Lat,edges.Node2Lat];
    else
        test_lon = [edges.Node1Lon,edges.Node2Lon];
        test_lat = [edges.Node1Lat,edges.Node2Lat];
        for idx = 1:span-1
            por = idx/(span-idx);
            test_lon = [test_lon porDiv(por,edges.Node1Lon,edges.Node2Lon)];
            test_lat = [test_lat porDiv(por,edges.Node1Lat,edges.Node2Lat)];
        end
    end
     test_lon = sort(test_lon); test_lat = sort(test_lat);
end