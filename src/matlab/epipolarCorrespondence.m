function [pts2] = epipolarCorrespondence(im1, im2, F, pts1)
% Find epipolar corresponding points.
%
% Args:
%   im1:    Image 1.
%   im2:    Image 2.
%   F:      Fundamental Matrix from im1 to im2.
%   pts1:   coordinates of points in image 1.
%
% Returns:
%   pts2:   coordinates of points in image 2.
%
% Author: Zi Jun, Xu
% Last modified: 05/26/19
    
    % Convert to double type (still RGB).
    im1_double = im2double(im1);
    im2_double = im2double(im2);

    nPts1 = size(pts1, 1);
    epi_line = F * [pts1, ones(nPts1, 1)]';
    % epi_line: N * 3.
    epi_line = epi_line';

    % Compute the endpoints of each epipolar line.
    points = findBorderPoints(epi_line, size(im1));

    KERNEL_SIZE = 11;
    BOUND = (KERNEL_SIZE - 1) / 2;
    pts2 = zeros(nPts1, 2);
    for i = 1: nPts1
        pts1_mask = ...
            im1_double(round(pts1(i, 2) - BOUND): round(pts1(i, 2) + BOUND), ...
            round(pts1(i, 1) - BOUND): round(pts1(i, 1) + BOUND), :);
        plist = bresenham(points(i, 1), points(i, 2), points(i, 3), points(i, 4));
        
        valid_plist = selectInBoundary(im2_double, plist, KERNEL_SIZE);

        pts2(i, :) = findCorrespondence(im2_double, valid_plist, pts1_mask, BOUND);
    end
end

function pts = findBorderPoints(epi_line, img_size)
    nPts = size(epi_line, 1);
    pts = -ones(nPts, 4);
    
    upper_bound = 0.5;
    bot_bound = upper_bound + img_size(1);
    left_bound = 0.5;
    right_bound = left_bound + img_size(2);

    % Compute the intersections of each line and the borders of the image.
    for i = 1: nPts
        end_p = zeros(1, 4);
        j = 1;

        % Check the left and right borders.
        row = -(epi_line(i, 1) * left_bound + epi_line(i, 3)) / epi_line(i, 2);
        if row >= upper_bound && row <= bot_bound
            end_p(j: j + 1) = [row; left_bound];
            j = j + 2;
        end

        row = -(epi_line(i, 1) * right_bound + epi_line(i, 3)) / epi_line(i, 2);
        if row >= upper_bound && row <= bot_bound
            end_p(j: j + 1) = [row; right_bound];
            j = j + 2;
        end

        % Check the upper and bottom borders.
        if j < 4
            col = -(epi_line(i, 2) * upper_bound + epi_line(i, 3)) / epi_line(i, 1);
            if col >= left_bound && col <= right_bound
                end_p(j: j + 1) = [upper_bound; col];
                j = j + 2;
            end
        end

        if j < 4
            col = -(epi_line(i, 2) * bot_bound + epi_line(i, 3)) / epi_line(i, 1);
            if col >= left_bound && col <= right_bound
                end_p(j: j + 1) = [bot_bound; col];
                j = j + 2;
            end
        end

        for k = j: 4
            end_p(k) = -1;
        end

        pts(i, :) = end_p([2, 1, 4, 3]);
    end
end

function correspondence = findCorrespondence(im2, plist, pts1_mask, bound)
    % pts: N * 2.
    nPlist = size(plist, 1);

    error_func = zeros(1, nPlist);
    for i = 1: nPlist
        pts2_mask = im2(plist(i, 2) - bound: plist(i, 2) + bound, ...
            plist(i, 1) - bound: plist(i, 1) + bound, :);      
        
        % Euclidean distance error function.
        error = (pts2_mask - pts1_mask) .^ 2;
        error_func(i) = sqrt(sum(error(:)));
    end
    
    [~, min_p] = min(error_func);
    correspondence = plist(min_p, :);
end

function valid_plist = selectInBoundary(img, pts, kernel_size)
    [lenIm, widthIm, ~] = size(img);
    bound = (kernel_size - 1) / 2;
    bot_bound = lenIm - bound;
    right_bound = widthIm - bound;

    nPts = size(pts, 1);
    lower = 1;
    upper = nPts;

    % Traverse from the beginning and stop at the points pairs that are in the bound.
    for i = 1: ceil(nPts / 2)
        if pts(i, 1) > bound && pts(i, 2) > bound
            lower = i;
            break;
        end
    end
    
    % Traverse from the end and stop at the points pairs that are not int the bound.
    for i = nPts: -1: ceil(nPts / 2)
        if pts(i, 1) < right_bound && pts(i, 2) < bot_bound
            upper = i;
            break;
        end
    end

    valid_plist = pts(lower: upper, :);
end

function p = bresenham(x1, y1, x2, y2) 
    x1 = round(x1); 
    x2 = round(x2);
    y1 = round(y1); 
    y2 = round(y2);

    % Compute x-direction and y-direction change.
    slope_x = abs(x2 - x1);
    slope_y = abs(y2 - y1);
    
    % If slope > 1, swap slope_x and slope_y.
    if slope_y > slope_x
        t = slope_x;
        slope_x = slope_y;
        slope_y = t; 
    end
    
    % Bresenham's line algorithm.
    if slope_y == 0
        q = zeros(slope_x + 1, 1);
    else
        tmp = floor(slope_x / 2): -slope_y: -slope_y * slope_x + floor(slope_x / 2);
        q = [0; diff(mod(tmp', slope_x)) >= 0];
    end

    if slope_y > slope_x
        if y1 <= y2
            y = [y1: y2]';
        else
            y = [y1: -1: y2]';
        end

        if x1 <= x2
            x = x1 + cumsum(q);
        else
            x = x1 - cumsum(q);
        end
    else
        if x1 <= x2
            x = [x1: x2]';
        else
            x = [x1: -1: x2]';
        end
        
        if y1 <= y2
            y = y1 + cumsum(q);
        else
            y = y1 - cumsum(q);
        end
    end
    
    p = [x, y];
end
