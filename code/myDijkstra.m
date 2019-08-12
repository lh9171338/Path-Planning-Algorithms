function [PathList, Mask] = myDijkstra(Map, startPoint, targetPoint, option)
%MYDIJKSTRA - Dijkstra path planning algorithm
%
%   PathList = myDijkstra(Map, startPoint, targetPoint)
%   PathList = myDijkstra(Map, startPoint, targetPoint, option)
%   [PathList, Mask] = myDijkstra(...)
%
%   option: 
%       'Diagonal': true/false, default(true)
%       'OccupyThresh': 0~1, default([])
%       'InflateRadius': integer, default([])


%% �������
narginchk(3,4);
nargoutchk(1,2);

%% �������ֵ���
% ����option
if nargin < 4
    option = struct('Diagonal', true, 'OccupyThresh', [], 'InflateRadius', []);
end
neighbor4 = [-1, 0; ...
             0, -1; 0, 1;...
             1, 0];
neighbor8 = [-1, -1; -1, 0; -1, 1;...
              0, -1;         0, 1;...
              1,-1; 1, 0; 1, 1];
if option.Diagonal
    neighbor = neighbor8;
else
    neighbor = neighbor4;
end

%% ͼ��Ԥ����
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

%% ����LabelMap
[height, width] = size(Map);
LabelMap = zeros(height, width);
obstacle = 0;       % 0: Obstacle
free = 1;           % 1: free
inOpenList = 2;     % 2: in the open list
inClosedList = 3;   % 3: in the closed lit
LabelMap((Map == 0)) = obstacle; 
LabelMap((Map == 1)) = free; 

%% Dijkstra·���滮
OpenList = struct('point', [], 'dist', []);
ParentMap = zeros(height, width);
finish = false;
startX = startPoint(1);
startY = startPoint(2);
targetX = targetPoint(1);
targetY = targetPoint(2);
% ��startPointѹ��OpenList
OpenList.point = cat(1, OpenList.point, startPoint);
OpenList.dist = cat(1, OpenList.dist, 0);
ParentMap(startY, startX) = 0;
LabelMap(startY, startX) = inOpenList;

%% �������е㣬ֱ������Ŀ�괦
while ~isempty(OpenList.dist)
    %% �ҵ�OpenList�о�����С�ĵ�
    [mindist, index] = min(OpenList.dist);
    curX = OpenList.point(index, 1);
    curY = OpenList.point(index, 2);
    OpenList.point(index,:) = [];
    OpenList.dist(index,:) = [];
    LabelMap(curY, curX) = inClosedList;
        
    %% �ж��Ƿ񵽴�Ŀ���
    if curX == targetX && curY == targetY
        finish = true;
        break;
    end
    
    %% ������ǰ�������
    for k = 1:length(neighbor)
        y = curY + neighbor(k, 1);
        x = curX + neighbor(k, 2);
       %% �ж��Ƿ����
        if y <= 0 || y > height || x <= 0 || x > width 
            continue;
        end      
        if LabelMap(y, x) == free || LabelMap(y, x) == inOpenList
           %% ���Խ����ܷ�ͨ��
            if option.Diagonal 
                walkable = true;
                if y == curY - 1 && LabelMap(curY - 1, curX) == obstacle
                    walkable = false;
                elseif y == curY + 1 && LabelMap(curY + 1, curX) == obstacle
                    walkable = false;
                end
                if x == curX - 1 && LabelMap(curY, curX - 1) == obstacle
                    walkable = false;
                elseif x == curX + 1 && LabelMap(curY, curX + 1) == obstacle
                    walkable = false;
                end
                if ~walkable
                    continue;
                end
            end
            
           %% �������
            if abs(x - curX) == 1 && abs(y - curY) == 1
                adddist = 14;
            else
                adddist = 10;
            end
            dist = mindist + adddist;
                
           %% ���¾���
            if LabelMap(y, x) == free
                OpenList.point = cat(1, OpenList.point, [x, y]);
                OpenList.dist = cat(1, OpenList.dist, dist);
                ParentMap(y, x) = sub2ind([height, width], curY, curX);
                LabelMap(y, x) = inOpenList;
            else % LabelMap(y, x) = inOpenList 
                index = find(all(OpenList.point == [x, y], 2));
                if dist < OpenList.dist(index)
                    OpenList.dist(index) = dist;
                    ParentMap(y, x) = sub2ind([height, width], curY, curX);
                end
            end  
        end
    end
end

%% ����·��
PathList = []; 
if finish
    PathList = cat(1, targetPoint, PathList);
    curX = targetX;
    curY = targetY;
    while ~( curX == startX &&  curY == startY)
        index = ParentMap(curY, curX);
        [curY, curX] = ind2sub([height, width], index);
        PathList = cat(1, [curX, curY], PathList);
    end
else
    disp('Can not find a path form startPoint to targetPoint!');
end


