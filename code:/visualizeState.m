function visualizeState(fig, ax, state, params, assignments, step)
% Stable viz (single axes): fixed limits, contours, robots on top.

if isempty(fig) || ~isvalid(fig), fig = figure('Color','w','Visible','on'); end
if isempty(ax)  || ~isvalid(ax),  ax  = axes('Parent',fig); end
figure(fig);  % bring to front

% Lock limits/aspect
xDom = [params.domain(1,1) params.domain(1,2)];
yDom = [params.domain(2,1) params.domain(2,2)];
set(ax,'XLimMode','manual','YLimMode','manual','YDir','normal');
xlim(ax,xDom); ylim(ax,yDom);
axis(ax,'equal'); pbaspect(ax,[1 1 1]);

% Clear and redraw
cla(ax); hold(ax,'on');

% Heatmap (priority)
himg = imagesc(ax, state.X(1,:), state.Y(:,1), state.priority');
colormap(ax,'hot'); set(ax,'CLimMode','manual');
set(himg,'AlphaData',0.5);

% Contours (demand)
contour(ax, state.X, state.Y, state.demand, 12, ...
        'LineColor',[0 0.35 0], 'LineWidth',1);

% Robots and paths
N = numel(state.Robots);
for i=1:N
    pth = state.paths{i};
    plot(ax, pth(:,1), pth(:,2), '-', ...
         'Color',[state.Robots(i).color 0.5], 'LineWidth',1);

    plot(ax, state.Robots(i).p(1), state.Robots(i).p(2), 'o', ...
         'MarkerSize',8, 'MarkerFaceColor',state.Robots(i).color, ...
         'MarkerEdgeColor','k');

    % capacity bar
    frac = state.Robots(i).capacity / params.capacity;
    w = 2.5; h = 0.8;
    x0 = state.Robots(i).p(1) - w/2;
    y0 = state.Robots(i).p(2) + 1.3;
    rectangle(ax,'Position',[x0 y0 w h], 'EdgeColor',[0 0 0], 'FaceColor','none');
    rectangle(ax,'Position',[x0 y0 w*max(0,min(1,frac)) h], ...
              'EdgeColor','none', 'FaceColor',[0 0.7 0]);
end

% Bases
plot(ax, params.bases(:,1), params.bases(:,2), 's', ...
     'MarkerFaceColor',[0 0 1], 'MarkerEdgeColor','k', 'MarkerSize',8);

% ----- Hotspot emergency labels (remaining demand per hotspot) -----
numH = size(params.hotspots,1);
for h = 1:numH
    cx  = params.hotspots(h,1);
    cy  = params.hotspots(h,2);
    rad = 2 * params.hotspots(h,3);
    mask = (state.X - cx).^2 + (state.Y - cy).^2 <= rad^2;

    remaining = sum(state.demand(mask));
    frac = remaining / max(state.hotspot_init(h), 1e-6);   % 0–1

    lbl = sprintf('H%d: %3.0f%%', h, 100*frac);          % e.g. H1: 72%

    % Place label slightly above the hotspot
    text(cx, cy + 0.7*rad, lbl, ...
        'Color','w', 'FontWeight','bold', ...
        'FontSize',10, 'HorizontalAlignment','center');
end


% Title
md = inf; Pxy = reshape([state.Robots.p], 2, []).';
for a=1:size(Pxy,1)
    for b=a+1:size(Pxy,1)
        d = norm(Pxy(a,:)-Pxy(b,:)); if d<md, md = d; end
    end
end
title(ax, sprintf('t = %.1f s | unmet demand = %.1f | min dist = %.2f m', ...
      state.t, sum(state.demand(:)), md));

hold(ax,'off');
end
