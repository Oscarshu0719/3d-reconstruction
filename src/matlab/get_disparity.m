function dispM = get_disparity(im1, im2, maxDisp, windowSize)
% Creates a disparity map from a pair of rectified images im1 and im2, 
%   given the maximum disparity MAXDISP and the window size WINDOWSIZE.
%
% Author: Zi Jun, Xu
% Last modified: 05/26/19

    mask = ones(windowSize, windowSize);
    
    [lenIm, widthIm] = size(im1);
    disparity = zeros(lenIm, widthIm, maxDisp + 1);
    tmp = zeros(lenIm, widthIm);

    for i = 0: maxDisp
        j = 1: (lenIm * (widthIm - i));
        tmp(j) = (im1(j + lenIm * i) - im2(j)) .^ 2;
        disparity(:, :, i + 1) = conv2(tmp, mask, 'same');
    end

    [~, min_pos] = min(disparity, [], 3);
    
    dispM = min_pos - 1;
end