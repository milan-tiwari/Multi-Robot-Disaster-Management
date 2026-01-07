function priority = generateField(P, X, Y)
% Priority field built from explicit hotspots in P.hotspots.

priority = zeros(size(X));

% P.hotspots rows: [cx cy sigma amplitude]
for k = 1:size(P.hotspots,1)
    cx = P.hotspots(k,1);
    cy = P.hotspots(k,2);
    s  = P.hotspots(k,3);
    a  = P.hotspots(k,4);
    priority = priority + a * exp(-((X-cx).^2 + (Y-cy).^2)/(2*s^2));
end

% Normalize to [0,1]
priority = priority - min(priority(:));
priority = priority ./ max(priority(:) + eps);
end
