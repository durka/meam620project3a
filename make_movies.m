%% phase 1

plot_ccapt(ccapt([1 1; 2 2], [2 1; 1 2], 0.5, 1, 0.01), 0.5, 0.01, 'movies/ccapt2D_closeproximity.avi');

%% phase 2

record = 'movies/ccaptQuad_random_waypts.avi';
runsim;
