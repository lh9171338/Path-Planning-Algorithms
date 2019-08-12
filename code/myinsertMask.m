function RGB = myinsertMask(I, Mask, color)
%MYINSERTMASK - Insert mask in image
%
%   This MATLAB function returns a truecolor image with mask inserted.
%
%   RGB = myinsertMask(I, Mask, color)
%

%% 参数检查
narginchk(3,3);
nargoutchk(1,1);

%% 绘制Mask
if size(I, 3) == 1
    I = repmat(I, [1,1,3]);
end
R = I(:, :, 1);
G = I(:, :, 2);
B = I(:, :, 3);
R(Mask) = color(1);
G(Mask) = color(2);
B(Mask) = color(3);
RGB = cat(3, R, G, B);

