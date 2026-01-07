function [cands_xy, cands_lin, cands_val] = taskSelection(i, idx, demand, X, Y, P, K)
% Return up to K candidate hotspots inside robot i's Voronoi cell, sorted by demand.

if nargin<7 || isempty(K), K = max(1, P.hotspot_topK); end

mask = (idx == i);
if ~any(mask,'all')
    cands_xy  = [];
    cands_lin = [];
    cands_val = [];
    return;
end

vals = demand; 
vals(~mask) = -inf;

vals_vec = vals(:);
[sorted_val, sorted_lin] = sort(vals_vec,'descend');

th = P.hotspot_frac_threshold * max(demand(:));
keep = isfinite(sorted_val) & (sorted_val > th);
sorted_lin = sorted_lin(keep);
sorted_val = sorted_val(keep);

K = min(K, numel(sorted_lin));
sorted_lin = sorted_lin(1:K);
sorted_val = sorted_val(1:K);

cands_lin = sorted_lin(:).';
cands_val = sorted_val(:).';

cands_xy = zeros(numel(cands_lin),2);
for k = 1:numel(cands_lin)
    [r,c] = ind2sub(size(demand), cands_lin(k));
    cands_xy(k,:) = [X(r,c) Y(r,c)];
end
end
