function F = eightpoint(pts1, pts2)
% Eight-point algorithm.
%
% Args:
%   pts1: N x 2 matrix of (x, y) coordinates.
%   pts2: N x 2 matrix of (x, y) coordinates.
%   M:    max(imwidth, imheight).
%
% Returns:
%   F:    Fundamental matrix mapping from pts1 to pts2.
%
% Author: Zi Jun, Xu
% Last modified: 05/25/19

%% Normalization.
    nPts1 = size(pts1, 1);
    centroid_1 = mean(pts1);
    centroid_2 = mean(pts2);
    
    trans_1 = pts1 - centroid_1;
    trans_2 = pts2 - centroid_2;

    scale_1 = sqrt(2) / mean(sqrt(sum(trans_1' .^ 2)));
    scale_2 = sqrt(2) / mean(sqrt(sum(trans_2' .^ 2)));
    
    T_1 = [scale_1, 0, -scale_1 * centroid_1(1);
         0, scale_1, -scale_1 * centroid_1(2);
         0, 0, 1];
    T_2 = [scale_2, 0, -scale_2 * centroid_2(1);
         0, scale_2, -scale_2 * centroid_2(2);
         0, 0, 1];
    
     % 3 * N.
    norm_1 = T_1 * [pts1'; ones(1, nPts1)];
    norm_2 = T_2 * [pts2'; ones(1, nPts1)];
    
%% Direct Linear Transformation (DLT).
    A = zeros(nPts1, 9);
    for i = 1: nPts1
       A(i, :) = [norm_1(1, i) * norm_2(1, i), norm_1(2, i) * norm_2(1, i), ... 
                  norm_2(1, i), norm_1(1, i) * norm_2(2, i), ...
                  norm_1(2, i) * norm_2(2, i), norm_2(2, i), norm_1(1, i), ...
                  norm_1(2, i), 1];
    end
    
%% Singular Value Decomposition (SVD).
    [~, ~, V] = svd(A, 0);
    F = reshape(V(:, 9), 3, 3)';
    
    [U, S, V] = svd(F);
    S(3, 3) = 0;
    F = U * S * V';
    
    F = T_2' * F * T_1;
    F = F / norm(F);
    
    if F(3, 3) < 0
        F = -F;
    end
end
