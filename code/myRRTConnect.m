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


%% 参数检查
narginchk(3,4);
nargoutchk(1,2);

%% 输入参数值检查
% 参数option
if nargin < 4
    option = struct('MaxIter', 1000, 'RandomSampleThresh', 0.5, 'StepLength', 20,...
        'OccupyThresh', [], 'InflateRadius', [], 'Display', false, 'PauseTime', 0.1);
end

%% 图像预处理
map = Map;
% 转单通道
if size(Map,3) == 3
    Map = rgb2gray(Map);
end
% 二值化
if isempty(option.OccupyThresh) % 未给出二值化阈值，使用大津法进行二值化
    option.OccupyThresh = graythresh(Map);
end
Map = imbinarize(Map, option.OccupyThresh);
% 膨胀
src = Map;
if ~isempty(option.InflateRadius)
    se = strel('disk', option.InflateRadius);
    Map = imerode(Map, se);
end
Mask = xor(src, Map);

%% 判断起始点
PathList = []; 
if ~feasiblePoint(startPoint, Map)
    disp('startPoint lies on an obstacle or outside map'); 
    return;
end
if ~feasiblePoint(targetPoint, Map)
    disp('targetPoint lies on an obstacle or outside map'); 
    return;
end

%% RTT路径规划
RRTree1 = struct('point', [], 'parent', []); % 根节点位于起点的RRTree
RRTree2 = struct('point', [], 'parent', []); % 根节点位于终点的RRTree
% 把startPoint添加到RRTree1
RRTree1.point = cat(1, RRTree1.point, startPoint);
RRTree1.parent = cat(1, RRTree1.parent, 0);
% 把targetPoint添加到RRTree2
RRTree2.point = cat(1, RRTree2.point, targetPoint);
RRTree2.parent = cat(1, RRTree2.parent, 0);

%% 扩展RRTree
[height, width] = size(Map);
swap_flag = false;
finish_flag = false;
if option.Display
    figure;
    imshow(map);
end
for iter = 1:option.MaxIter
    %% 生成采样点
    if rand <= option.RandomSampleThresh
        sample = rand(1, 2) .* [width, height];   % 随机采样
    else
        sample = targetPoint;                    % 选择目标点
    end
        
    %% 选择RRTree1中距采样点最近的节点
    [~, parentIndex] = min(distanceCost(RRTree1.point, sample)); 
    closestNode = RRTree1.point(parentIndex, :); 
    
    %% 让节点朝着采样点生长
    orin = atan2(sample(2) - closestNode(2), sample(1) - closestNode(1));  
    newPoint = round(closestNode + option.StepLength * [cos(orin), sin(orin)]);
    if ~checkPath(closestNode, newPoint, Map) % 判断是否可以到达新节点
        continue;
    end
    
    %% 更新RRTree1
    [~, index] = min(distanceCost(RRTree1.point, newPoint)); 
    if index ~= parentIndex % 判断newPoint的StepLength范围内是否已经存在节点
        continue;
    else                        % 把newPoint添加到RRTree
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
        
       %% 选择RRTree2中距newPoint最近的节点
        [~, parentIndex] = min(distanceCost(RRTree2.point, newPoint)); 
        closestNode = RRTree2.point(parentIndex, :);
        
       %% 让closestNode节点朝着newPoint生长
        while true
           %% 判断RRTree1与RRTree2是否相连
            if distanceCost(closestNode, newPoint) < option.StepLength && checkPath(closestNode, newPoint, Map)
                finish_flag = true;
                RRTree2.point = cat(1, RRTree2.point, newPoint);
                RRTree2.parent = cat(1, RRTree2.parent, parentIndex);  
                break;
            end
            
           %% 生长closestNode
            orin = atan2(newPoint(2) - closestNode(2), newPoint(1) - closestNode(1));  
            newPoint2 = round(closestNode + option.StepLength * [cos(orin), sin(orin)]); 
            if ~checkPath(closestNode, newPoint2, Map)  % 判断是否可以到达新节点
                break;
            else                                        % 生长RRTree2
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
        
       %% 如果RRTree1小于RRTree2，则进行交换
        if length(RRTree2.point) < length(RRTree1.point)
            temp = RRTree2;
            RRTree2 = RRTree1;
            RRTree1 = temp;
            swap_flag = ~swap_flag;
        end
    end
end

%% 回推路径
if swap_flag % 保证RRTree1根节点为startPoint，RRTree2根节点为targetPoint
    temp = RRTree2;
    RRTree2 = RRTree1;
    RRTree1 = temp; 
end
if finish_flag
    %% 查找RRTree1的路径
    point = RRTree1.point(end, :);
    PathList = cat(1, point, PathList);
    index = RRTree1.parent(end);
    while index > 0 % 
        point = RRTree1.point(index, :);
        PathList = cat(1, point, PathList);
        index = RRTree1.parent(index);
    end
    %% 查找RRTree2的路径
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
