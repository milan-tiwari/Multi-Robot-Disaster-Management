function main()
% ReLIEF‑VOR: Disaster‑response coverage + relief delivery with multi‑robots

clc; clear;  % comment out 'close all' while debugging
% close all;

params = defaultParams();
rng(params.seed);

state = initScenario(params);
state = runSimulation(state, params);

disp('Simulation complete.');
end
