function S = initScenario(P)
% Build grid, priority field, initial demand, robots, and metrics.

% Grid
x = linspace(P.domain(1,1), P.domain(1,2), P.ngrid(1));
y = linspace(P.domain(2,1), P.domain(2,2), P.ngrid(2));
[X, Y] = meshgrid(x, y);

priority = generateField(P, X, Y);         % [0,1]
demand0  = priority * P.demand_scale;
demand   = demand0;

% Hotspot initial demand (for per-hotspot emergency levels)
numH = size(P.hotspots,1);
S.hotspot_init = zeros(numH,1);
for h = 1:numH
    cx  = P.hotspots(h,1);
    cy  = P.hotspots(h,2);
    rad = 2 * P.hotspots(h,3);           % radius ~ 2 sigma
    mask = (X - cx).^2 + (Y - cy).^2 <= rad^2;
    S.hotspot_init(h) = sum(demand0(mask));
end


% Robots
N  = P.N;
R  = struct('p', [], 'v', [], 'capacity', P.capacity, ...
            'state', "coverage", 'target', [NaN NaN], ...
            'color', [], 'base_id', 1);
Robots = repmat(R, N, 1);

% place near base(s)
nbases = size(P.bases,1);
for i = 1:N
    bID = min(nbases, max(1, round(i*N/(N+1)))); % rough split if many bases
    center = P.bases(bID,:);
    Robots(i).p = center + 2*randn(1,2);
    Robots(i).v = [0 0];
    Robots(i).capacity = P.capacity;
    Robots(i).color = rand(1,3);
    Robots(i).base_id = bID;
end

% Metrics buffers
S.metrics.t          = [];
S.metrics.J          = [];
S.metrics.unmet      = [];
S.metrics.minDist    = [];
S.metrics.coveredFrac= [];

% Paths for plotting
S.paths = cell(N,1);
for i=1:N, S.paths{i} = Robots(i).p; end

% Pack state
S.t = 0;
S.X = X; S.Y = Y;
S.priority = priority;
S.demand0  = demand0;
S.demand   = demand;

S.Robots = Robots;
S.N      = N;
S.P      = P;

% Hotspot threshold (absolute)
S.hotspot_abs_threshold = P.hotspot_frac_threshold * max(demand0(:));

% Output dir
outdir = char(P.outdir);
if ~exist(outdir,'dir'); mkdir(outdir); end
end
