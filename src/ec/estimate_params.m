function [K, R, t] = estimate_params(P)
% Computes the intrinsic K, rotation R and translation t.
% Args:
%   P: camera matrix.
%
% Returns:
%   K: Intrinsics matrix.
%   R: Rotation matrix.
%   t: Translation matrix.
%
% Author: Zi Jun, Xu
% Last modified: 05/26/19
    
    paradiagonal = [0, 0, 1; 0, 1, 0; 1, 0, 0];
    
    A = paradiagonal * P(:, 1: 3);
    % QR decomposition.
    [tmp_Q, tmp_R] = qr(A');

    R = paradiagonal * tmp_Q';
    K = paradiagonal * tmp_R' * paradiagonal;
    t = -R * -inv(P(:, 1: 3)) * P(:, 4);
end