function [PathList, Mask] = myRRT(Map, startPoint, targetPoint, option)
%MYRRT - RRT path planning algorithm
%
%   PathList = myRRT(Map, startPoint, targetPoint)
%   PathList = myRRT(Map, startPoint, targetPoint, option)
%   [PathList, Mask] = myRRT(...)
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
RRTree = struct('point', [], 'parent', []);
% ��startPoint��ӵ�RRTree
RRTree.point = cat(1, RRTree.point, startPoint);
RRTree.parent = cat(1, RRTree.parent, 0);

%% ��չRRTree
finish = false;
[height, width] = size(Map);
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
        
    %% ѡ��RRTree�о����������Ľڵ�
    [~, parentIndex] = min(distanceCost(RRTree.point, sample)); 
    closestNode = RRTree.point(parentIndex, :);
    
    %% �ж��Ƿ�ﵽĿ���
    if distanceCost(closestNode, targetPoint) < option.StepLength && checkPath(closestNode, targetPoint, Map) 
        finish = true;
        RRTree.point = cat(1, RRTree.point, targetPoint);
        RRTree.parent = cat(1, RRTree.parent, parentIndex);
        break;
    end    
    
    %% �ýڵ㳯�Ų���������
    orin = atan2(sample(2) - closestNode(2), sample(1) - closestNode(1));  
    newPoint = round(closestNode + option.StepLength * [cos(orin), sin(orin)]);
    if ~checkPath(closestNode, newPoint, Map) % �ж��Ƿ���Ե����½ڵ�
        continue;
    end
    
    %% ����RRTree
    [~, index] = min(distanceCost(RRTree.point, newPoint)); 
    if index ~= parentIndex % �ж�newPoint��StepLength��Χ���Ƿ��Ѿ����ڽڵ�
        continue;
    else                        % ��newPoint��ӵ�RRTree
        RRTree.point = cat(1, RRTree.point, newPoint);
        RRTree.parent = cat(1, RRTree.parent, parentIndex);
        if option.Display
            map = insertShape(map, 'Line', [closestNode, newPoint], 'Color', [255,0,0], 'LineWidth', 2);
            imshow(map);
            pause(option.PauseTime);
        end
    end
    
end

%% ����·��
if finish
    PathList = cat(1, targetPoint, PathList);
    index = RRTree.parent(end);
    while index ~= 0
        point = RRTree.point(index, :);
        PathList = cat(1, point, PathList);
        index = RRTree.parent(index);
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
