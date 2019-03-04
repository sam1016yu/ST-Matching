function GPSData = splitGPS2line(filename, delta_T, tolerance_p)
% GPS�ָ��
% 
% ��Ҫ���̣�
% 1. �����Ե�һ��15����Ϊһ�飬����ʱ����
% 2. ���ʱ������������������Ϊ��15����Ϊһ���켣��ֱ�Ӽ�¼
% 3. �����15���㲻������������ÿ2�����ǰ�������м��㣬������ֵ��λ�ã��Ӵ˴��Ͽ�
% 4. ��ʼ���㣬�ڶϿ�����ǰ�Ĺ켣��ʱ�����Ƿ���������
% 5. ���������������������Ͽ����һ�أ�ֱ����������Ϊֹ
% 6. �������������������Ͽ����������죬�ж��Ƿ�����������������㣬�������죬ֱ������������λ��
% 7. ��ʱ�����ǿ��Եõ���������������ֵ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Import necessary data
GPSData = importGPSfile(filename); % �˴�ʹ��importdata�Զ����ɵĴ��룬��ʱ�Ƚϳ�
%% �ָ�
GPSData = sortrows(GPSData);   %��GPS���ݸ���GPSDeviceID�������򣬽�ͬһID�����ݼ��е�һ��

% ����õ�GPSDevice IDһ�����ļ���
% ���⣬��Ϊ���Ѿ���������numFirst��ʾ���ID��һ�γ��ֵ�λ�ã�numLast��ʾ���ID���һ�γ��ֵ�λ�ã����ڷָͬGPSdeviceID
[Index1,numFirst] = unique(GPSData(:,1));
[~,numLast] = unique(GPSData(:,1), 'legacy');


%% Plot the import result on the figure.
% �ⲿ����������ͬ��GPSDevice ID�ò�ͬ����ɫ����ͼ�ϣ���Ϊ�ٶȽ������Ҳ���Ҫ������ע��
% colormap = rand([length(Index1(:,1)), 3]);
% figure;
% for i = 1:length(Index1)
%     hold on
%     for j = numFirst(i):numLast(i)
%         plot(GPSData(:,3)(j), GPSData(:,4)(j), '*', 'color', colormap(i,:))
%     end
% end


%% adjust time to meet the standard

%trace = [];         % traceΪ���յĹ켣�����
                    % ��һ��ΪGPSDevice ID
                    % �ڶ���Ϊ��ʼ�ļ�¼��GPSData�����ڵ�λ��
                    % ������Ϊ�����ļ�¼��GPSData�����ڵ�λ��
                    
                    % Update: ��Ϊֱ����ԭ������������һ�н��в����������Ժ���
                    
% diffstatus = [];    % �洢����֮��Ĳ��ģ��м����ݣ����á�

Tag = 1;

% ���ÿһ��GPSDevice ID����һ��forѭ��
for i = 1:length(Index1)
    
    beginnum = numFirst(i); % �켣�ĳ�ʼ��
    while (beginnum < numLast(i))                 % �ж���û�н���
        if beginnum < numLast(i) - 14             % �ж�ʣ����Ƿ�����15���Թ��и�
            range = 15;
        else
            range = numLast(i) - beginnum + 1;
        end
        
        % �����������ֵ��ʵ�ʵ�ʱ����
        max_tolerance = tolerance_p * delta_T * (range - 1);
        T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
        % ���ʱ�����޷�����
        if T > max_tolerance
%             disp(GPSData(:,2)(beginnum:beginnum+range))
            % ��ǰ�������ʱ������������
            diffArray = diff(GPSData(beginnum:beginnum+range-1,2));
            
%             while length(diffArray) < 14
%                 diffArray(end+1) = 0;
%             end
%             diffstatus(end+1, :) = diffArray;
            % ��֪ʱ���������������λ�ã����Ҿݴ���ɵ�һ���и�
            [~, range] = max(diffArray);
            % ���¼����������ֵ��ʵ�ʵ�ʱ����
            max_tolerance = tolerance_p * delta_T * (range - 1);
            T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
            % �ж��Ƿ��ܹ����ܣ�����������ԣ���������
            while  T > max_tolerance
                range = range - 1;
                max_tolerance = tolerance_p * delta_T * (range - 1);
                T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
            end
            % �ж��Ƿ��ܹ����š���������ԣ���������
            while T <= max_tolerance
                range = range + 1;
                if beginnum + range - 1 > numLast(i)
                    break
                end
                max_tolerance = tolerance_p * delta_T * (range - 1);
                T = GPSData(beginnum + range - 1,2) - GPSData(beginnum,2);
            end
            range = range - 1; %������������
            if beginnum + range - 1 > numLast(i)
                range = numLast(i) - beginnum + 1;
            end
        end
        % ��� �洢���ݣ�
        GPSData(beginnum:beginnum + range - 1,7) = Tag;
        Tag = Tag + 1;
%         trace(end+1, 1) = targetID;
%         trace(end, 2) = beginnum;
%         trace(end, 3) = beginnum + range - 1;
%         trace(end, 4) = GPSData(:,3)(beginnum);
%         trace(end, 5) = GPSData(:,4)(beginnum);
%         trace(end, 6) = GPSData(:,3)(beginnum + range - 1);
%         trace(end, 7) = GPSData(:,4)(beginnum + range - 1);
        beginnum = beginnum + range; %�����ε����һ�������һ���㣬��Ϊ�¸��켣�ĵ�һ���㡣
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