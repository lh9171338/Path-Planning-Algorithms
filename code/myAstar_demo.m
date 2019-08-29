%% myAstar_demo
clear,clc;
close all;

%% 读取地图图片
% Map = imread('../maps/gmapping.jpg');
% Map = imread('../maps/maze.jpg');
Map = imread('../maps/load-2.jpg');
if size(Map, 3) == 1
    Map = repmat(Map, 1, 1, 3);
end

%% 路径规划
figure;imshow(Map);
waitforbuttonpress;
point = get(gca,'CurrentPoint');
startPoint = round(point(1, 1:2)) + 1;
waitforbuttonpress;
point = get(gca,'CurrentPoint');
targetPoint = round(point(1, 1:2)) + 1;
option = struct('Euclidean', false, 'OccupyThresh', [], 'InflateRadius', 1);
tic;
[PathList, Mask] = myAstar(Map, startPoint, targetPoint, option);
toc

%% 显示结果
Map = myinsertMask(Map, Mask, [0,0,255]);
PathList(:,3:4) = 1;
Map = insertShape(Map, 'Rectangle', PathList, 'Color', [255,0,0], 'LineWidth', 2);
Map = insertShape(Map, 'Rectangle', PathList(1, :), 'Color', [0,255,0], 'LineWidth', 10);
Map = insertShape(Map, 'Rectangle', PathList(end, :), 'Color', [0,0,255], 'LineWidth', 10);
figure;imshow(Map);

