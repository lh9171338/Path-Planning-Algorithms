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
figure;imshow(Map);
waitforbuttonpress;
point = get(gca,'CurrentPoint');
startPoint = round(point(1, 1:2)) + 1;
waitforbuttonpress;
point = get(gca,'CurrentPoint');
targetPoint = round(point(1, 1:2)) + 1;
option = struct('MaxIter', 1000000, 'RandomSampleThresh', 0.5, 'StepLength', 5,...
    'OccupyThresh', [], 'InflateRadius', 5, 'Display', true, 'PauseTime', 0.001);
tic;
[PathList, Mask] = myRRTConnect(Map, startPoint, targetPoint, option);
toc

%% 显示结果
%figure;imshow(Map);
Map = myinsertMask(Map, Mask, [0,0,255]);
for i = 1:length(PathList) - 1
    Map = insertShape(Map, 'line', [PathList(i, :), PathList(i + 1, :)], 'LineWidth', 2, 'Color', [255,0,0]);
end
figure;imshow(Map);

