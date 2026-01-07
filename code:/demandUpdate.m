function [demand, Robots] = demandUpdate(demand, Robots, P, X, Y, dt)
% Apply delivery around robots that are serving, reduce capacity accordingly,
% and diffuse/decay the demand field.

for i=1:numel(Robots)
    if Robots(i).state == "serve" && Robots(i).capacity > 0
        dx = X - Robots(i).p(1);
        dy = Y - Robots(i).p(2);
        r  = sqrt(dx.^2 + dy.^2);
        mask = r <= P.service_radius;

        delivered = P.service_rate * dt; 
        if Robots(i).capacity < delivered
            delivered = Robots(i).capacity;
        end

        if any(mask,'all') && delivered > 0
            local = demand(mask);
            total_local = sum(local) + 1e-6;
            frac = delivered * (local / total_local);
            demand(mask) = max(0, local - frac);
            Robots(i).capacity = max(0, Robots(i).capacity - delivered);
        end

        if Robots(i).capacity <= 0
            Robots(i).state = "to_base";
        end
    end
end

% Diffusion / decay
if P.diffuse_sigma > 0
    demand = smoothField(demand, P.diffuse_sigma);
end
if P.diffuse_decay > 0
    demand = P.diffuse_decay * demand;
end
end
