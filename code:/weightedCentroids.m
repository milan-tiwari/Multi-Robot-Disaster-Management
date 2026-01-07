function C = weightedCentroids(idx, W, X, Y, N)
% Priority/demand-weighted centroids per Voronoi cell (discrete grid).

C = nan(N,2);
for i=1:N
    mask = (idx == i);
    wsum = sum(W(mask));
    if wsum > 0
        cx = sum(X(mask).*W(mask)) / wsum;
        cy = sum(Y(mask).*W(mask)) / wsum;
        C(i,:) = [cx cy];
    end
end
end
