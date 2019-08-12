function [PathList, Mask] = myAstar(Map, startPoint, targetPoint, option)
%MYASTAR - A star path planning algorithm
%
%   PathList = myAstar(Map, startPoint, targetPoint)
%   PathList = myAstar(Map, startPoint, targetPoint, option)
%   [PathList, Mask] = myAstar(...)
%
%   option: 
%       'Euclidean': true/false, default(true)
%       'OccupyThresh': 0~1, default([])
%       'InflateRadius': integer, default([])

%% �������
narginchk(3,4);
nargoutchk(1,2);

%% �������ֵ���
% ����option
if nargin < 4
    option = struct('Euclidean', true, 'OccupyThresh', [], 'InflateRadius', []);
end
neighbor = [-1, -1; -1, 0; -1, 1;...
             0, -1;         0, 1;...
             1,-1; 1, 0; 1, 1];
          
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

%% A*·���滮
OpenList = struct('point', [], 'value', []);
finish = false;
startX = startPoint(1);
startY = startPoint(2);
targetX = targetPoint(1);
targetY = targetPoint(2);
% ��startPointѹ��OpenList
OpenList.point = cat(1, OpenList.point, startPoint);
OpenList.value = cat(1, OpenList.value, [0, 0, 0]);
ParentMap(startY, startX) = 0;
LabelMap(startY, startX) = inOpenList;

%% �������е㣬ֱ������Ŀ�괦
while ~isempty(OpenList)
    %% �ҵ�OpenList��Fֵ��С�ĵ�
    [~, index] = min(OpenList.value(:,1));
    Cost = OpenList.value(index,:);
    curX = OpenList.point(index, 1);
    curY = OpenList.point(index, 2);
    OpenList.point(index,:) = [];
    OpenList.value(index,:) = [];
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
            
           %% ������ۺ���
            % ����Gcost
            if abs(x - curX) == 1 && abs(y - curY) == 1
                addedGcost = 14;
            else
                addedGcost = 10;
            end
            Gcost = Cost(2) + addedGcost;
            % ����Hcost
            if option.Euclidean % ����ŷʽ����
                Hcost = 10 * sqrt((x - targetX)^2 + (y - targetY)^2);
            else                % ���������پ���
                Hcost = 10 * (abs(x - targetX) + abs(y - targetY));
            end
            % ����Fcost
            Fcost = Gcost + Hcost;
                
           %% ���´��ۺ���
            if LabelMap(y, x) == free
                % ����ǰ��ѹ��OpenList
                OpenList.point = cat(1, OpenList.point, [x, y]);
                OpenList.value = cat(1, OpenList.value, [Fcost, Gcost, Hcost]);
                ParentMap(y, x) = sub2ind([height, width], curY, curX);
                LabelMap(y, x) = inOpenList;
            else % LabelMap(y, x) = inOpenList 
                % �Ƚ�Gcost
                index = find(all(OpenList.point == [x, y], 2));
                if Gcost < OpenList.value(index, 2)
                    OpenList.value(index,:) = [Fcost, Gcost, Hcost];
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


