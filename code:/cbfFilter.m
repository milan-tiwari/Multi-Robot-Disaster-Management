function u = cbfFilter(i, Pxy, Vxy, u_nom, params)
% CBF-style safety filter with repulsive fallback for pairwise separation.
% Inputs:
%   i      - index of robot being filtered
%   Pxy    - Nx2 positions
%   Vxy    - Nx2 velocities (unused in simple repulsion)
%   u_nom  - 1x2 nominal control
%   params - struct from defaultParams

p_i = Pxy(i,:);
N = size(Pxy,1);

useQP = params.cbf_use_quadprog && exist('quadprog','file')==2;
Ai = []; bi = [];

% Quadratic CBF constraints (if we have quadprog)
for j=1:N
    if j==i, continue; end
    p_j = Pxy(j,:);
    v_j = Vxy(j,:); %#ok<NASGU>
    d = norm(p_i - p_j);
    if d < params.safety_range
        h = d^2 - params.d_min^2;
        a = 2*(p_i - p_j);
        b = -params.cbf_alpha*h; % simple version (ignoring neighbor vel)
        Ai = [Ai; -a]; 
        bi = [bi; -b];
    end
end

if useQP && ~isempty(Ai)
    H = eye(2);
    f = -u_nom(:);
    try
        u = quadprog(H, f, Ai, bi, [], [], ...
            [-params.speed_max -params.speed_max], ...
            [ params.speed_max  params.speed_max], [], ...
            optimoptions('quadprog','Display','off'));
        u = u(:).';
        return;
    catch
        % fall through to repulsion
    end
end

% Smooth repulsive fallback
u = u_nom;
for j=1:N
    if j==i, continue; end
    p_j = Pxy(j,:);
    d = norm(p_i - p_j) + 1e-6;
    if d < params.safety_range
        gain = params.repulse_gain * max(0, params.d_min - d) / d;
        u = u + gain * (p_i - p_j);
    end
end
end
