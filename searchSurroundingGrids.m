function [road_ids] = searchSurroundingGrids(grid_size,center_cell_id,search_steps,grid)
%searchSurroundingGrids 
%   search surrounding edges for candidate edges
%   Input:
%   grid_size: m*n array
%   search_steps: steps that search is conducted
%   grid: grided road network info

% consider looking around center grid based on the given search radius and
% grid size used previously
grid_cols = grid_size(2);grid_rows = grid_size(1);

% function that convert between cellId and  [row_id,col_id]
% convenient for searching surrounding grids
cell2rowcol = @(cellid) [ceil(cellid/grid_cols), ...
    mod(cellid,grid_cols)+(mod(cellid,grid_cols)==0)*grid_cols];
rowcol2cell = @(row,col) grid_cols*(row-1) + col; 
isValidRowCol = @(row,col) row>0 && row<=grid_rows && col>0 && col <= grid_cols;

road_ids = [];
center_rowcol = cell2rowcol(center_cell_id);
center_row = center_rowcol(1); center_col = center_rowcol(2);
if search_steps > 0
    for step_col = -search_steps:search_steps
        for step_row = -search_steps:search_steps
            if isValidRowCol(center_row+step_row,center_col+step_col)
                road_ids = [road_ids cell2mat(grid(rowcol2cell(center_row+step_row,center_col+step_col),:).roadID)];
            end
        end
    end
    road_ids = unique(road_ids);
else
    road_ids = cell2mat(grid(center_cell_id,:).roadID);
end
end

