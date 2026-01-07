function metrics = measureMetrics(state, params, assignments)
% Compute locational cost J, unmet demand, min separation, and coverage fraction.

X = state.X; Y = state.Y;
Pfield = state.priority;
D = state.demand;
robots = state.Robots;

Rxy = reshape([robots.p], 2, []).'; % Nx2
gx = X(:); gy = Y(:);
W = Pfield(:);

% nearest distance squared to a robot for each grid point
if nargin < 3 || isempty(assignments)
    D2 = zeros(numel(gx), size(Rxy,1));
    for i=1:size(Rxy,1)
        D2(:,i) = (gx-Rxy(i,1)).^2 + (gy-Rxy(i,2)).^2;
    end
    [d2,~] = min(D2,[],2);
else
    d2 = zeros(numel(gx),1);
    for i=1:size(Rxy,1)
        mask = (assignments==i);
        tmp = (X(mask)-Rxy(i,1)).^2 + (Y(mask)-Rxy(i,2)).^2;
        d2(mask) = tmp;
    end
end
J = sum(W .* d2);

% Unmet demand
unmet = sum(D(:));

% Min pairwise distance
minDist = inf;
for i=1:numel(robots)
    for j=i+1:numel(robots)
        dij = norm(robots(i).p - robots(j).p);
        if dij < minDist, minDist = dij; end
    end
end

% Coverage fraction served
S0 = sum(state.demand0(:));
coveredFrac = 1 - unmet / max(S0,1e-6);

metrics = struct('J',J, 'unmet',unmet, ...
                 'minDist',minDist, 'coveredFrac',coveredFrac);
end
