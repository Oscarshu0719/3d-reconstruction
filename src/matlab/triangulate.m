function [pts3d, error] = triangulate(P1, pts1, P2, pts2)
% Estimate the 3D positions of points from 2d correspondence.
% Args:
%   P1:     Projection matrix with shape 3 x 4 for image 1.
%   pts1:   Coordinates of points with shape N x 2 on image 1.
%   P2:     Projection matrix with shape 3 x 4 for image 2.
%   pts2:   Coordinates of points with shape N x 2 on image 2.
%
% Returns:
%   Pts3d:  Coordinates of 3D points with shape N x 3.
%
% Author: Zi Jun, Xu
% Last modified: 05/25/19

    nPts1 = size(pts1, 1);
    nPts2 = size(pts2, 1);

    % 3 * N.
    homo_pts1 = [pts1'; ones(1, nPts1)];
    homo_pts2 = [pts2'; ones(1, nPts2)];

    pts3d = zeros(4, nPts1);
    for i = 1: nPts1
        % Direct Linear Transformation (DLT).
        x = [0, homo_pts1(3, i), -homo_pts1(2, i); ...
            -homo_pts1(3, i), 0, homo_pts1(1, i); ...
            homo_pts1(2, i), -homo_pts1(1, i), 0];
        y = [0, homo_pts2(3, i), -homo_pts2(2, i);...
            -homo_pts2(3, i), 0, homo_pts2(1, i);...
            homo_pts2(2, i), -homo_pts2(1, i), 0];

        A = [x * P1; y * P2];
        [~, ~, V] = svd(A);

        pts3d(:, i) = V(:, end) / V(end, end);
    end
    
    % 3 * N.
    pts1_hat = P1 * pts3d;
    pts1_hat(1, :) = pts1_hat(1, :) / pts1_hat(3, :);
    pts1_hat(2, :) = pts1_hat(2, :) / pts1_hat(3, :);
    pts1_hat(3, :) = 1;

    pts2_hat = P2 * pts3d;
    pts2_hat(1, :) = pts2_hat(1, :) / pts2_hat(3, :);
    pts2_hat(2, :) = pts2_hat(2, :) / pts2_hat(3, :);
    pts2_hat(3, :) = 1;

    error = norm(homo_pts1 - pts1_hat) ^ 2 + norm(homo_pts2 - pts2_hat) ^ 2;
    
    pts3d = pts3d(1: 3, :)';
end
