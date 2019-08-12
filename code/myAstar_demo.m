%% myAstar_demo
clear,clc;
close all;

%% 读取地图图片
% Map = imread('../maps/gmapping.jpg');
% Map = imread('../maps/maze.jpg');
Map = imread('../maps/load.jpg');
if size(Map, 3) == 1
    Map = repmat(Map, 1, 1, 3);
end

%% 路径规划
% startPoint = [1300, 1000];
% targetPoint = [1750, 920];
% startPoint = [20, 20];
% targetPoint = [480, 480];
startPoint = [160, 379];
targetPoint = [710, 259];
option = struct('Euclidean', false, 'OccupyThresh', [], 'InflateRadius', []);
tic;
[PathList, Mask] = myAstar(Map, startPoint, targetPoint, option);
toc

%% 显示结果
figure;imshow(Map);

Map = myinsertMask(Map, Mask, [0,0,255]);
PathList(:,3:4) = 1;
Map = insertShape(Map, 'Rectangle', PathList, 'Color', [255,0,0], 'LineWidth', 2);
Map = insertShape(Map, 'Rectangle', PathList(1, :), 'Color', [0,255,0], 'LineWidth', 10);
Map = insertShape(Map, 'Rectangle', PathList(end, :), 'Color', [0,0,255], 'LineWidth', 10);
figure;imshow(Map);

