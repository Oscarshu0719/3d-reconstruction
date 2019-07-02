function depthM = get_depth(dispM, K1, K2, R1, R2, t1, t2)
% Creates a depth map from a disparity map (DISPM).
%
% Author: Zi Jun, Xu
% Last modified: 05/26/19

    [lenIm, widthIm] = size(dispM);
    depthM = zeros(lenIm, widthIm);

    c1 = (K1 * R1) \ (K1 * t1);
    c2 = (K2 * R2) \ (K2 * t2);
    b = norm(c1 - c2);
    f = K1(1, 1);

    for i = 1: lenIm
        for j = 1: widthIm
            if(dispM(i, j) == 0)
                depthM(i, j) = 0;
            else
                depthM(i, j) = b * f / dispM(i, j);
            end
        end
    end
end