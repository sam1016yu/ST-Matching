function GPSData = splitGPS2line(filename, delta_T, tolerance_p)
% GPS分割处理
% 
% 主要流程：
% 1. 首先以第一个15个点为一组，计算时间间隔
% 2. 如果时间间隔满足条件，则认为这15个点为一个轨迹，直接记录
% 3. 如果这15个点不满足条件，对每2个点的前后间隔进行计算，获得最大值的位置，从此处断开
% 4. 开始计算，在断开处以前的轨迹的时间间隔是否满足条件
% 5. 如果不满足条件，则继续断开最后一截，直到满足条件为止
% 6. 如果满足条件，则尝试向断开处进行延伸，判断是否满足条件，如果满足，继续延伸，直到不满足条件位置
% 7. 此时，我们可以得到满足条件的最大的值。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Import necessary data
GPSData = importGPSfile(filename); % 此处使用importdata自动生成的代码，耗时比较长
%% 分割
GPSData = sortrows(GPSData);   %对GPS数据根据GPSDeviceID进行排序，将同一ID的数据集中到一起

% 计算得到GPSDevice ID一共有哪几个
% 另外，因为我已经做了排序，numFirst表示这个ID第一次出现的位置，numLast表示这个ID最后一次出现的位置，用于分割不同GPSdeviceID
[Index1,numFirst] = unique(GPSData(:,1));
[~,numLast] = unique(GPSData(:,1), 'legacy');


%% Plot the import result on the figure.
% 这部分用来将不同的GPSDevice ID用不同的颜色画在图上，因为速度较慢，且不需要，所以注释
% colormap = rand([length(Index1(:,1)), 3]);
% figure;
% for i = 1:length(Index1)
%     hold on
%     for j = numFirst(i):numLast(i)
%         plot(GPSData(:,3)(j), GPSData(:,4)(j), '*', 'color', colormap(i,:))
%     end
% end


%% adjust time to meet the standard

%trace = [];         % trace为最终的轨迹结果，
                    % 第一列为GPSDevice ID
                    % 第二列为开始的记录在GPSData中所在的位置
                    % 第三列为结束的记录在GPSData中所在的位置
                    
                    % Update: 改为直接在原数据中添加最后一列进行操作，便于以后处理
                    
% diffstatus = [];    % 存储两点之间的差别的，中间数据，无用。

Tag = 1;

% 针对每一个GPSDevice ID进行一个for循环
for i = 1:length(Index1)
    
    beginnum = numFirst(i); % 轨迹的初始点
    while (beginnum < numLast(i))                 % 判断有没有结束
        if beginnum < numLast(i) - 14             % 判断剩余点是否满足15个以供切割
            range = 15;
        else
            range = numLast(i) - beginnum + 1;
        end
        
        % 计算最大容忍值和实际的时间间隔
        max_tolerance = tolerance_p * delta_T * (range - 1);
        T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
        % 如果时间间隔无法容忍
        if T > max_tolerance
%             disp(GPSData(:,2)(beginnum:beginnum+range))
            % 对前后两点的时间进行求差运算
            diffArray = diff(GPSData(beginnum:beginnum+range-1,2));
            
%             while length(diffArray) < 14
%                 diffArray(end+1) = 0;
%             end
%             diffstatus(end+1, :) = diffArray;
            % 得知时间间隔最大的两个点的位置，并且据此完成第一次切割
            [~, range] = max(diffArray);
            % 重新计算最大容忍值和实际的时间间隔
            max_tolerance = tolerance_p * delta_T * (range - 1);
            T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
            % 判断是否能够忍受，如果还不可以，继续缩减
            while  T > max_tolerance
                range = range - 1;
                max_tolerance = tolerance_p * delta_T * (range - 1);
                T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
            end
            % 判断是否能够扩张。如果还可以，继续扩张
            while T <= max_tolerance
                range = range + 1;
                if beginnum + range - 1 > numLast(i)
                    break
                end
                max_tolerance = tolerance_p * delta_T * (range - 1);
                T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
            end
            range = range - 1; %满足扩张需求
            if beginnum + range - 1 > numLast(i)
                range = numLast(i) - beginnum + 1;
            end
        end
        % 最后， 存储数据，
        GPSData(beginnum:beginnum + range - 1,7) = Tag;
        Tag = Tag + 1;
%         trace(end+1, 1) = targetID;
%         trace(end, 2) = beginnum;
%         trace(end, 3) = beginnum + range - 1;
%         trace(end, 4) = GPSData(:,3)(beginnum);
%         trace(end, 5) = GPSData(:,4)(beginnum);
%         trace(end, 6) = GPSData(:,3)(beginnum + range - 1);
%         trace(end, 7) = GPSData(:,4)(beginnum + range - 1);
        beginnum = beginnum + range; %将本次的最后一个点的下一个点，设为下个轨迹的第一个点。
    end
end
end

function GPSData = importGPSfile(filename)
delimiter = ',';
%% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%[^\n\r]';
%% Open the text file.
fileID = fopen(filename,'r');
%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter',delimiter, 'TextType',...
    'string', 'EmptyValue', NaN,  'ReturnOnError', false);
%% Close the text file.
fclose(fileID);
%% Create output variable
GPSData = [dataArray{1:end-1}];
end

%% Clear temporary variables
% clearvars beginnum delta_T diffArray filename i Index1 max_tolerance num numFirst numLast range T Tag targetID tolerance_p; 