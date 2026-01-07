function A = smoothField(A, sigma)
% Lightweight Gaussian smoothing using conv2.

if sigma <= 0, return; end

sz = max(3, 2*ceil(3*sigma)+1);
half = (sz-1)/2;
[x,y] = meshgrid(-half:half, -half:half);
g = exp(-(x.^2 + y.^2)/(2*sigma^2));
g = g / sum(g(:));

A = conv2(A, g, 'same');
end
