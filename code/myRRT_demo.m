%% myRRT_demo
clear,clc;
close all;

%% 读取地图图片
%Map = imread('../maps/gmapping.jpg');
Map = imread('../maps/maze.jpg');
if size(Map, 3) == 1
    Map = repmat(Map, 1, 1, 3);
end

%% 路径规划
% startPoint = [1300, 1000];
% targetPoint = [1750, 920];
startPoint = [20, 20];
targetPoint = [480, 480];
option = struct('MaxIter', 1000000, 'RandomSampleThresh', 0.5, 'StepLength', 5,...
    'OccupyThresh', [], 'InflateRadius', 5, 'Display', false, 'PauseTime', 0.001);
tic;
[PathList, Mask] = myRRT(Map, startPoint, targetPoint, option);
toc

%% 显示结果
% figure;imshow(Map);
Map = myinsertMask(Map, Mask, [0,0,255]);
for i = 1:length(PathList) - 1
    Map = insertShape(Map, 'line', [PathList(i, :), PathList(i + 1, :)], 'LineWidth', 2, 'Color', [255,0,0]);
end
figure;imshow(Map);

