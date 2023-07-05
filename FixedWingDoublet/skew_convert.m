function aX = skew_convert(a)

% skew_convert.m
%
% Obtains the skew symmetric matrix [aX] of [a] such that [a]X[b] = [aX][b] 

%By definition of skew-symmetric marix
aX = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
