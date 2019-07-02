function P = estimate_pose(x, X)
% Computes the pose matrix (camera matrix) P given 2D and 3D points.
% 
% Args:
%   x: 2D points with shape [2, N]
%   X: 3D points with shape [3, N]
% 
% Returns:
%   P: Camera matrix.
%
% Author: Zi Jun, Xu
% Last modified: 05/26/19
    
    % N * 4.
    homo_X = [X; ones(1, size(X, 2))]';
    % N * 3.
    homo_x = [x; ones(1, size(x, 2))]';

    x1 = zeros(size(homo_X, 1) * 2, 4 * 2);
    x1(1: 2: end, 1: 4) = homo_X(:, :);
    x1(2: 2: end, 5: 8) = homo_X(:, :);

    x2 = zeros(size(homo_x, 1) * 2, 4);
    x2(1: 2: end, 1) = homo_x(:, 1);
    x2(2: 2: end, 1) = homo_x(:, 2);
    x2(:, 2) = x2(:, 1);
    x2(:, 3) = x2(:, 1);
    x2(:, 4) = x2(:, 1);
    
    tmp_x1 = x1(:, 1: 4);
    tmp_x1(2: 2: end, :) = tmp_x1(1: 2: end, :);
    x2 = x2 .* tmp_x1;

    A = [-x1, x2];
    [~, ~, V] = svd(A);

    P = reshape(V(:, end), [4, 3])';
end