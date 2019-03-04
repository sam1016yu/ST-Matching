function [road_network,road_type,road_cells,road_ids,grid_size] = splitRoad2Cell(filename,cell_size)
% filename: the raw txt file of road network, in required format
% grid size: the desired grid size (in km)
%% read raw text file of road network
delimiter = ',';
formatSpec = '%f%f%f%f%f%f%f%C%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter',delimiter, 'TextType',...
    'string', 'EmptyValue', NaN,  'ReturnOnError', false);
fclose(fileID);
road_network_raw = [dataArray{1:end-2}];
road_type = dataArray{end-1};
clearvars delimiter formatSpec fileID dataArray ans;
%% cut grids
% determining 4 points on the edges
lon_max = max(max(road_network_raw(:,4)),max(road_network_raw(:,6)));
lon_min = min(min(road_network_raw(:,4)),min(road_network_raw(:,6)));
lat_max = max(max(road_network_raw(:,5)),max(road_network_raw(:,7)));
lat_min = min(min(road_network_raw(:,5)),min(road_network_raw(:,7)));
% calculate area size
area_height = max(deg2km(distance([lat_max,lon_min],[lat_min,lon_min])),...
    deg2km(distance([lat_max,lon_max],[lat_min,lon_max])));
area_width = max(deg2km(distance([lat_max,lon_max],[lat_max,lon_min])),...
    deg2km(distance([lat_min,lon_max],[lat_min,lon_min])));
% calculate number of grids
y_size = ceil(area_height/cell_size);x_size = ceil(area_width/cell_size);
% lon_vec:grid lines(longitude), lat_vec grid lines(latitude)
lon_vec = linspace(lon_min,lon_max,x_size);
lat_vec = linspace(lat_min,lat_max,y_size);
%% find grid 
road_ids = findEdgeIndex(lon_vec,lat_vec,road_network_raw);
rows_ids = length(road_ids);road_cells = zeros(rows_ids,5);
% road_cells = table(roadID);
%% fill info of each cell
road_cells(:,1) = (1:rows_ids)';
road_cells(:,2) = repmat(lon_vec(1:end-1),1,y_size-1)';
road_cells(:,3) = repmat(lon_vec(2:end),1,y_size-1)';
road_cells(:,4) = reshape(repmat(lat_vec(1:end-1),x_size-1,1),[],1);
road_cells(:,5) = reshape(repmat(lat_vec(2:end),x_size-1,1),[],1);
%% final output
road_network = road_network_raw([1,4:end],:);
grid_size = [y_size-1, x_size-1];
end


function road_cells = findEdgeIndex(lon_vec,lat_vec,road_network_raw)
%% given gridded network and edge,find whcih grid edge belong to

% Initialize a waitbar to show progress
% wbh = waitbar(0,'indexing edges...');
road_cells = cell((length(lon_vec)-1) * (length(lat_vec)-1),1);
% since linspace() is used, cell width and height are identical
cell_width = lon_vec(2)-lon_vec(1);cell_height = lat_vec(2)-lat_vec(1);
[rows_road,~] = size(road_network_raw);
for edges_idx = 1: rows_road
%     fprintf('Indexing edges %i of %i to cut grid\n',edges_idx,rows_road);
    edges = road_network_raw(edges_idx,:);
    % for a given edge, call divideEdges() to generate test points
    [test_lon_vec,test_lat_vec] = divideEdges(edges,cell_width,cell_height);
    edge_index = zeros(1,length(test_lon_vec));
    for test_idx = 1:length(test_lon_vec)
        % for each test point, test its position in the lon_vec and lat_vec
        test_lon = test_lon_vec(test_idx);test_lat = test_lat_vec(test_idx);
        [~,col_lon,~] = find(lon_vec>=test_lon);
        [~,col_lat,~] = find(lat_vec>=test_lat);
        lon_idx = col_lon(1) -1;lat_idx = col_lat(1) -1;
        % deal with special occasion when points overlap edges
        if lon_idx == 0
            lon_idx = 1;
        elseif lat_idx == 0
            lat_idx = 1;
        end
        % calculate grid index
        % start from left to right top to bottom
        grid_index = lon_idx + (length(lon_vec)-1)*(lat_idx -1 );
        assert(grid_index > 0);
        % store into corresponding edges
        edge_index(test_idx) = grid_index;
    end
    % different test points can lie in the same grid,need to remove
    % duplicate
    edge_index = unique(edge_index);
    % For future usage, query which edges one grid contains.
    % Each edge can belong to different grids. 
    % Store info in the cell array road_cells. 
    for idx = 1:length(edge_index)
        grid_id = edge_index(idx);
        if isempty(road_cells{grid_id,1})
            road_cells{grid_id,1} = edges(:,1);
        else
            road_cells{grid_id,1} = [road_cells{grid_id,1} edges(:,1)];
        end
    end
    % update waitbar
%     waitbar(edges_idx/height(road_network_raw),wbh);
end
% close(wbh);
end


function [test_lon,test_lat] = divideEdges(edges,lon_grid,lat_grid)
%%
    % calculate coordiinates to be tested
    % e.g. if edge is divided into 3 portions,
    % test points are:
    % start, end, 1/3 and 2/3 division point
    % span of the search
    lon_span = ceil(abs(edges(:,4)-edges(:,6))/(lon_grid));
    lat_span = ceil(abs(edges(:,5)-edges(:,7))/(lat_grid));
    span = max(lon_span,lat_span);
    % definite proportional division point formula 定比分点公式
    porDiv = @(lambda,start_cor,end_cor) (start_cor+lambda*end_cor)/(1+lambda);
    % always include starting and ending points
    test_lon = [edges(:,4),edges(:,6)];
    test_lat = [edges(:,5),edges(:,7)];
    if span > 1
        test_lon = [test_lon zeros(1,span-1)];
        for idx = 1:span-1
            por = idx/(span-idx);
            test_lon(2+idx) = porDiv(por,edges(:,4),edges(:,6));
            test_lat(2+idx) = porDiv(por,edges(:,5),edges(:,7));
        end
    end
    test_lon = sort(test_lon); test_lat = sort(test_lat);
end