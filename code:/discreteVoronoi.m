function idx = discreteVoronoi(robotsXY, X, Y)
% Grid-based Voronoi: each grid cell gets index of nearest robot.

N = size(robotsXY,1);
gx = X(:); gy = Y(:);
G  = [gx gy];
M  = size(G,1);

D2 = zeros(M, N);
for i=1:N
    d = (G(:,1)-robotsXY(i,1)).^2 + (G(:,2)-robotsXY(i,2)).^2;
    D2(:,i) = d;
end

[~, I] = min(D2, [], 2);
idx = reshape(I, size(X));
end
