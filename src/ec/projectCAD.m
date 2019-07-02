% A test script using PnP.mat.
%
% Author: Zi Jun, Xu
% Last modified: 05/25/19

clc;
clear;
close all;

%% Compute the camera matrix.
load  '../data/PnP.mat';
P = estimate_pose(x, X);

%% Compute intrinsics, rotation matrix, and transition matrix.
[K_est, R_est, T_est] = estimate_params(P);

%% Estimated 3D points.
x_est = P * [X; ones(1, size(X, 2))];
x_est = ([x_est(1, :) ./ x_est(3, :); x_est(2, :) ./ x_est(3, :)])';
x = x';

%% Plot 2D and 3D points onto the image.
figure;
imshow(image);
hold on;

scatter(x_est(:, 1), x_est(:, 2), 10, 'r', 'o', 'filled');
scatter(x(:, 1), x(:, 2), 40, 'b', 'o');

%% Draw the rotated CAD model using `trimesh`.
cad_R_est = (R_est * (cad.vertices'))';

figure;
trimesh(cad.faces, cad_R_est(:, 1), cad_R_est(:, 2), cad_R_est(:, 3));

%% Draw the projected CAD model onto the image using `patch`.
cad_P = K_est * [R_est, T_est] * ...
    [cad.vertices, ones(size(cad_R_est, 1), 1)]';
cad_P = int16(([cad_P(1, :) ./ cad_P(3, :); cad_P(2, :) ./ cad_P(3, :)])');

figure;
imshow(image);
hold on;
p = patch('Faces', cad.faces, 'Vertices', cad_P, ...
    'FaceColor', 'yellow', 'EdgeColor', 'none');
p.FaceAlpha = 0.15;
