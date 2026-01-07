function state = runSimulation(state, P)
% Core loop: coverage -> tasking -> safety -> integrate -> demand update -> visualize

if ~exist(P.outdir,'dir'); mkdir(P.outdir); end
fig = []; ax = [];

steps = ceil(P.T / P.dt);
for k = 1:steps
    t = (k-1)*P.dt;
    state.t = t;
    N = state.N;

    % Positions and velocities
    Rxy = zeros(N,2);
    Vxy = zeros(N,2);
    for i=1:N
        Rxy(i,:) = state.Robots(i).p;
        Vxy(i,:) = state.Robots(i).v;
    end

    % Discrete Voronoi
    assignments = discreteVoronoi(Rxy, state.X, state.Y);

    % Coverage weights
    Dn = state.demand;
    if max(Dn(:)) > 0, Dn = Dn / max(Dn(:)); end
    W = state.priority + P.lambda_demand_in_coverage * Dn;

    % Centroids
    C = weightedCentroids(assignments, W, state.X, state.Y, N);

    % --------- UNIQUE HOTSPOT CLAIMING (greedy) ---------
    cand_xy = cell(N,1); cand_lin = cell(N,1); cand_val = cell(N,1);
    for i=1:N
        [xyi, lini, vali] = taskSelection(i, assignments, state.demand, ...
                                         state.X, state.Y, P, max(3,P.hotspot_topK));
        cand_xy{i}  = xyi;
        cand_lin{i} = lini;
        cand_val{i} = vali;
    end

    bestVals = -inf(1,N);
    for i=1:N
        vi = cand_val{i};
        if ~isempty(vi), bestVals(i) = vi(1); end
    end
    [~, order] = sort(bestVals, 'descend');

    claimed      = false(size(state.demand));
    chosenTarget = nan(N,2);
    hasTask      = false(1,N);

    for kk = 1:N
        i = order(kk);
        li = cand_lin{i}; xyi = cand_xy{i};
        for m = 1:numel(li)
            if ~claimed(li(m))
                claimed(li(m)) = true;
                chosenTarget(i,:) = xyi(m,:);
                hasTask(i) = true;
                break;
            end
        end
    end
    % -----------------------------------------------------

    % Nominal controls
    U_nom = zeros(N,2);
    for i=1:N
        % Coverage default
        ci = C(i,:);
        if all(isfinite(ci))
            u_cov = -P.k_cov * (state.Robots(i).p - ci);
        else
            u_cov = [0 0];
        end

        if hasTask(i) && state.Robots(i).capacity > 0
            state.Robots(i).state  = "to_task";
            state.Robots(i).target = chosenTarget(i,:);
            u_task = -P.k_task * (state.Robots(i).p - state.Robots(i).target);
            if norm(state.Robots(i).p - state.Robots(i).target) <= P.goal_tol
                state.Robots(i).state = "serve";
                u_task = [0 0];
            end
            u_nom = u_task;
        else
            if state.Robots(i).capacity <= 0
                % go to nearest base
                [~,bid] = min(vecnorm(P.bases - state.Robots(i).p, 2, 2));
                base = P.bases(bid,:);
                state.Robots(i).state  = "to_base";
                state.Robots(i).target = base;
                u_nom = -P.k_task * (state.Robots(i).p - base);
                if norm(state.Robots(i).p - base) <= P.base_arrival_tol
                    state.Robots(i).capacity = P.capacity;
                    state.Robots(i).state     = "coverage";
                end
            else
                state.Robots(i).state = "coverage";
                u_nom = u_cov;
            end
        end

        sp = norm(u_nom);
        if sp > P.speed_max, u_nom = (P.speed_max/sp) * u_nom; end
        U_nom(i,:) = u_nom;
    end

    % Safety filter
    U = zeros(N,2);
    for i=1:N
        U(i,:) = cbfFilter(i, Rxy, Vxy, U_nom(i,:), P);
        sp = norm(U(i,:));
        if sp > P.speed_max, U(i,:) = (P.speed_max/sp) * U(i,:); end
    end

    % Integrate
    for i=1:N
        state.Robots(i).v = U(i,:);
        state.Robots(i).p = state.Robots(i).p + P.dt * U(i,:);

        % keep in domain
        state.Robots(i).p(1) = min(max(state.Robots(i).p(1), P.domain(1,1)), P.domain(1,2));
        state.Robots(i).p(2) = min(max(state.Robots(i).p(2), P.domain(2,1)), P.domain(2,2));

        state.paths{i}(end+1,:) = state.Robots(i).p;
    end

    % Demand update
    [state.demand, state.Robots] = demandUpdate(state.demand, state.Robots, ...
                                                P, state.X, state.Y, P.dt);

    % Metrics
    m = measureMetrics(state, P, assignments);
    state.metrics.t(end+1)          = t;
    state.metrics.J(end+1)          = m.J;
    state.metrics.unmet(end+1)      = m.unmet;
    state.metrics.minDist(end+1)    = m.minDist;
    state.metrics.coveredFrac(end+1)= m.coveredFrac;

    % Visualization
    if mod(k-1, P.visualize_every) == 0
        if isempty(fig) || ~isvalid(fig), fig = figure('Color','w','Visible','on'); end
        if isempty(ax)  || ~isvalid(ax),  ax  = axes('Parent',fig); end
        visualizeState(fig, ax, state, P, assignments, k);
        drawnow;
    end
end

% Save summary
summaryPath = fullfile(P.outdir, 'sim_summary.mat');
metrics = state.metrics; %#ok<NASGU>
save(summaryPath, 'state', 'metrics');

% ---- Final metrics plots (cleaner styling, black text) ----
t = state.metrics.t;

f = figure('Color','w','Visible','on');
set(f, 'DefaultAxesFontSize', 12, ...
       'DefaultAxesLineWidth', 1.0, ...
       'DefaultTextColor', 'k', ...
       'DefaultAxesXColor', 'k', ...
       'DefaultAxesYColor', 'k');

subplot(2,2,1);
plot(t, state.metrics.J/1e6, 'LineWidth', 2);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k');
xlabel('time [s]','Color','k');
ylabel('J [\times 10^6]','Color','k');
title('Locational cost','Color','k');

subplot(2,2,2);
plot(t, state.metrics.unmet/1e5, 'LineWidth', 2);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k');
xlabel('time [s]','Color','k');
ylabel('Unmet demand [\times 10^5]','Color','k');
title('Unmet demand','Color','k');

subplot(2,2,3);
plot(t, state.metrics.coveredFrac*100, 'LineWidth', 2);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k');
xlabel('time [s]','Color','k');
ylabel('Coverage served [%]','Color','k');
title('Coverage served','Color','k');

subplot(2,2,4);
plot(t, state.metrics.minDist, 'LineWidth', 2);
yline(P.d_min, '--r','d_{min}','LineWidth',1);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k');
xlabel('time [s]','Color','k');
ylabel('Min pairwise dist [m]','Color','k');
title('Safety (min distance)','Color','k');

sgtitle('ReLIEF-VOR Metrics','Color','k');


end
