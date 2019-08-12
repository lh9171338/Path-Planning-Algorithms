function [PathList, Mask] = myRRTConnect(Map, startPoint, targetPoint, option)
%MYRRTCONNECT - RRT connect path planning algorithm
%
%   PathList = myRRTConnect(Map, startPoint, targetPoint)
%   PathList = myRRTConnect(Map, startPoint, targetPoint, option)
%   [PathList, Mask] = myRRTConnect(...)
%
%   option: 
%       'MaxIter': integer, default(1000)
%       'RandomSampleThresh': 0~1, default(0.5)
%       'StepLength': integer, default(20)
%       'OccupyThresh': 0~1, default([])
%       'InflateRadius': integer, default([])
%       'Display': bool, default(false)
%       'PauseTime': double, default(0.1)


%% �������
narginchk(3,4);
nargoutchk(1,2);

%% �������ֵ���
% ����option
if nargin < 4
    option = struct('MaxIter', 1000, 'RandomSampleThresh', 0.5, 'StepLength', 20,...
        'OccupyThresh', [], 'InflateRadius', [], 'Display', false, 'PauseTime', 0.1);
end

%% ͼ��Ԥ����
map = Map;
% ת��ͨ��
if size(Map,3) == 3
    Map = rgb2gray(Map);
end
% ��ֵ��
if isempty(option.OccupyThresh) % δ������ֵ����ֵ��ʹ�ô�򷨽��ж�ֵ��
    option.OccupyThresh = graythresh(Map);
end
Map = imbinarize(Map, option.OccupyThresh);
% ����
src = Map;
if ~isempty(option.InflateRadius)
    se = strel('disk', option.InflateRadius);
    Map = imerode(Map, se);
end
Mask = xor(src, Map);

%% �ж���ʼ��
PathList = []; 
if ~feasiblePoint(startPoint, Map)
    disp('startPoint lies on an obstacle or outside map'); 
    return;
end
if ~feasiblePoint(targetPoint, Map)
    disp('targetPoint lies on an obstacle or outside map'); 
    return;
end

%% RTT·���滮
RRTree1 = struct('point', [], 'parent', []); % ���ڵ�λ������RRTree
RRTree2 = struct('point', [], 'parent', []); % ���ڵ�λ���յ��RRTree
% ��startPoint��ӵ�RRTree1
RRTree1.point = cat(1, RRTree1.point, startPoint);
RRTree1.parent = cat(1, RRTree1.parent, 0);
% ��targetPoint��ӵ�RRTree2
RRTree2.point = cat(1, RRTree2.point, targetPoint);
RRTree2.parent = cat(1, RRTree2.parent, 0);

%% ��չRRTree
[height, width] = size(Map);
swap_flag = false;
finish_flag = false;
if option.Display
    figure;
    imshow(map);
end
for iter = 1:option.MaxIter
    %% ���ɲ�����
    if rand <= option.RandomSampleThresh
        sample = rand(1, 2) .* [width, height];   % �������
    else
        sample = targetPoint;                    % ѡ��Ŀ���
    end
        
    %% ѡ��RRTree1�о����������Ľڵ�
    [~, parentIndex] = min(distanceCost(RRTree1.point, sample)); 
    closestNode = RRTree1.point(parentIndex, :); 
    
    %% �ýڵ㳯�Ų���������
    orin = atan2(sample(2) - closestNode(2), sample(1) - closestNode(1));  
    newPoint = round(closestNode + option.StepLength * [cos(orin), sin(orin)]);
    if ~checkPath(closestNode, newPoint, Map) % �ж��Ƿ���Ե����½ڵ�
        continue;
    end
    
    %% ����RRTree1
    [~, index] = min(distanceCost(RRTree1.point, newPoint)); 
    if index ~= parentIndex % �ж�newPoint��StepLength��Χ���Ƿ��Ѿ����ڽڵ�
        continue;
    else                        % ��newPoint��ӵ�RRTree
        RRTree1.point = cat(1, RRTree1.point, newPoint);
        RRTree1.parent = cat(1, RRTree1.parent, parentIndex);
        if option.Display
            if ~swap_flag
                color = [255,0,0];
            else
                color = [0,255,0];
            end
            map = insertShape(map, 'Line', [closestNode, newPoint], 'Color', color, 'LineWidth', 2);
            imshow(map);
            pause(option.PauseTime);
        end
        
       %% ѡ��RRTree2�о�newPoint����Ľڵ�
        [~, parentIndex] = min(distanceCost(RRTree2.point, newPoint)); 
        closestNode = RRTree2.point(parentIndex, :);
        
       %% ��closestNode�ڵ㳯��newPoint����
        while true
           %% �ж�RRTree1��RRTree2�Ƿ�����
            if distanceCost(closestNode, newPoint) < option.StepLength && checkPath(closestNode, newPoint, Map)
                finish_flag = true;
                RRTree2.point = cat(1, RRTree2.point, newPoint);
                RRTree2.parent = cat(1, RRTree2.parent, parentIndex);  
                break;
            end
            
           %% ����closestNode
            orin = atan2(newPoint(2) - closestNode(2), newPoint(1) - closestNode(1));  
            newPoint2 = round(closestNode + option.StepLength * [cos(orin), sin(orin)]); 
            if ~checkPath(closestNode, newPoint2, Map)  % �ж��Ƿ���Ե����½ڵ�
                break;
            else                                        % ����RRTree2
                RRTree2.point = cat(1, RRTree2.point, newPoint2);
                RRTree2.parent = cat(1, RRTree2.parent, parentIndex);  
                if option.Display
                    if ~swap_flag
                        color = [0,255,0];
                    else
                        color = [255,0,0];
                    end
                    map = insertShape(map, 'Line', [closestNode, newPoint2], 'Color', color, 'LineWidth', 2);
                    imshow(map);
                    pause(option.PauseTime);
                end
                closestNode = newPoint2;
                parentIndex = length(RRTree2.point);
            end 
        end
        
        if finish_flag
            break;
        end
        
       %% ���RRTree1С��RRTree2������н���
        if length(RRTree2.point) < length(RRTree1.point)
            temp = RRTree2;
            RRTree2 = RRTree1;
            RRTree1 = temp;
            swap_flag = ~swap_flag;
        end
    end
end

%% ����·��
if swap_flag % ��֤RRTree1���ڵ�ΪstartPoint��RRTree2���ڵ�ΪtargetPoint
    temp = RRTree2;
    RRTree2 = RRTree1;
    RRTree1 = temp; 
end
if finish_flag
    %% ����RRTree1��·��
    point = RRTree1.point(end, :);
    PathList = cat(1, point, PathList);
    index = RRTree1.parent(end);
    while index > 0 % 
        point = RRTree1.point(index, :);
        PathList = cat(1, point, PathList);
        index = RRTree1.parent(index);
    end
    %% ����RRTree2��·��
    index = RRTree2.parent(end);
    while index > 0 % 
        point = RRTree2.point(index, :);
        PathList = cat(1, PathList, point);
        index = RRTree2.parent(index);
    end    
else
    fprintf('Can not find a path form startPoint to targetPoint! Iteration number: %i\n', iter);
end

%% distanceCost
function d = distanceCost(a,b)
d = sqrt(sum((a - b).^2, 2));
	
%% feasiblePoint
function feasible = feasiblePoint(point, map)
feasible = true;
[height, width] = size(map);
x = point(1);
y = point(2);
% check if collission-free spot and inside maps
if ~(x > 0 && x <= width && y > 0 && y <= height && map(y, x) == 1)
    feasible = false;
end

%% checkPath
function feasible = checkPath(point, newPoint, map)
feasible = true;
orin = atan2(newPoint(2) - point(2), newPoint(1) - point(1));
delta = [cos(orin), sin(orin)];
dist = distanceCost(point, newPoint);
for r = 0:1:dist
    tempPoint = round(point + r * delta);
    if ~feasiblePoint(tempPoint, map)
        feasible = false;
        break;
    end
end
