function P = defaultParams()
% All tunable parameters for ReLIEF‑VOR (MATLAB)

% --- Simulation & domain ---
P.seed            = 7;                 % RNG seed
P.dt              = 0.5;               % [s] time step
P.T               = 180;               % [s] total duration
P.domain          = [0 100; 0 100];    % [xmin xmax; ymin ymax]
P.ngrid           = [80 80];           % grid resolution [nx ny]
P.visualize_every = 2;                 % draw every N steps (>=1)
P.show_voronoi    = false;             % (not used now, but kept)
P.save_frames     = false;             % if you want PNGs later

% --- Hotspots: [cx cy sigma amplitude] ---
% (three disaster sites, all on the left/center side; no bottom-right blob)
P.hotspots = [ 25 25  8 1.0;    % H1 bottom-left
               50 55 10 1.0;    % H2 center
               25 80  8 1.0;  % H3 upper-left
               80 30  8 1.0];


% --- Team ---
P.N               = 12;                % # robots
P.speed_max       = 3.0;               % [m/s]
P.bases           = [95 95];           % single depot (change if you like)

% --- Coverage (Lloyd descent) ---
P.k_cov           = 1.2;               % gain toward centroid
P.lambda_demand_in_coverage = 1.5;     % how much demand pulls coverage

% --- Tasking (hotspot selection & goal seeking) ---
P.k_task          = 2.0;               % goal‑seeking gain
P.goal_tol        = 1.5;               % [m] distance to "arrived" at hotspot
P.hotspot_topK    = 5;                 % consider up to K candidates/robot
P.hotspot_frac_threshold = 0.05;       % min frac of global max demand
P.base_arrival_tol = 1.5;              % [m] distance to "arrived" at base

% --- Demand (relief dynamics) ---
P.demand_scale    = 30.0;             % initial demand = priority * this
P.service_radius  = 2.5;               % [m] service influence radius
P.service_rate    = 30.0;               % [units/s] depletion at hotspot
P.capacity        = 600.0;             % [units] per robot
P.diffuse_sigma   = 0.0;               % demand smoothing sigma (0=off)
P.diffuse_decay   = 1.0;             % global decay per step

% --- Safety (CBF‑style / repulsion) ---
P.d_min           = 3.0;               % [m] minimum separation
P.safety_range    = 6.0;               % [m] neighbors considered
P.cbf_alpha       = 1.0;               % CBF alpha
P.cbf_use_quadprog = true;             % if Optimization Toolbox exists
P.repulse_gain    = 2.5;               % fallback repulsion gain

% --- Output ---
P.outdir          = "out";
end
