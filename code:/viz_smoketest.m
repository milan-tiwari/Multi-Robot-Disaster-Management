function viz_smoketest
params = defaultParams();
rng(params.seed);
S = initScenario(params);

% one-time Voronoi just for the call
Rxy = reshape([S.Robots.p], 2, []).';
assignments = discreteVoronoi(Rxy, S.X, S.Y);

fig = figure('Color','w','Visible','on'); ax = axes('Parent',fig);
visualizeState(fig, ax, S, params, assignments, 0);
end
