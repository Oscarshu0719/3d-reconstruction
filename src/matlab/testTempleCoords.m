% A test script using templeCoords.mat.
%
% Author: Zi Jun, Xu
% Last modified: 05/26/19

clc;
clear;
close all;

img_1 = imread('../data/im1.png');
img_2 = imread('../data/im2.png');

%% Compute the fundamental matrix from given corressponding points pairs in two images using eight-point algorithm.
load '../data/someCorresp.mat';
F = eightpoint(pts1, pts2);

% Test fundamental matri using bulit-in function.
% correct_F = estimateFundamentalMatrix(pts1, pts2, 'Method', 'Norm8Point');

% Show the result of `eightpoint` function.
figure;
displayEpipolarF(img_2, img_1, F);

%% Compute the points correspondences in the second image from given points pair in first image and the fundamental matrix.
load '../data/templeCoords.mat';
pts2 = epipolarCorrespondence(img_1, img_2, F, pts1);

% Test epipolar correspondences.
figure;
epipolarMatchGUI(img_1, img_2, F);

%% Compute the essential matrix from the fundamental matrix and the intrinsics of two cameras.
load '../data/intrinsics.mat';
E = essentialMatrix(F, K1, K2);

%% Compute the projection matrix of two cameras (P2 has four possibilities).
P1 = K1 * [eye(3), zeros(3, 1)];
candidates_M2 = camera2(E);

%% Triangulation.
nPts1 = size(pts1, 1);
candidates_pts2 = zeros(nPts1, 3, 4);
candidates_error = zeros(nPts1, 4);
for i = 1: 4
    [candidates_pts2(:, :, i), candidates_error(:, i)] = triangulate(P1, pts1, K2 * candidates_M2(:, :, i), pts2);
end

%% Choose the correct projection matrix of the second camera.
for i = 1: 4
    % Most points are in front of two cameras (positive depth).
    if all(candidates_pts2(:, 3, i) > 0)
        correspondence_pts2 = candidates_pts2(:, :, i);
        error = candidates_error(i);
        P2 = K2 * candidates_M2(:, :, i);
        break;
    end
end

% Show the reprojection error.
disp(['Reprojection error: ', num2str(error)]);

%% Plot points correspondences using `scatter3`.
figure;
scatter3(correspondence_pts2(:, 1), correspondence_pts2(:, 2), ...
    correspondence_pts2(:, 3), 'filled');

%% save extrinsic parameters for dense reconstruction
R1 = eye(3);
t1 = zeros(3, 1);

M2 = K2 \ P2;
R2 = M2(:, 1: 3);
t2 = R2 \ M2(:, 4);

save('../data/extrinsics.mat', 'R1', 't1', 'R2', 't2');
