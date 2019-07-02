function E = essentialMatrix(F, K1, K2)
% Computes the essential matrix.
%
% Args:
%   F:  Fundamental Matrix.
%   K1: Camera Matrix 1.
%   K2: Camera Matrix 2.
%
% Returns:
%   E:  Essential Matrix.
%
% Author: Zi Jun, Xu
% Last modified: 05/25/19

    E = K1' * F * K2;
end
