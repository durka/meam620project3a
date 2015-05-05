clearvars;

nquads = 25;
R = 0.5;
Vmax = 5;
H = 1.5;
tf = 5;
dt = 0.05;

plot_ccapt(ccapt(rand(nquads,2)*20, rand(nquads,2)*20, R, Vmax, dt), R,dt);

plot_ccapt3D(ccapt3D(rand(nquads,3)*20, rand(nquads,3)*20, R, Vmax, dt), R, dt);

dcapt2d(rand(nquads,2)*20, rand(nquads,2)*20, R, H, tf, dt, true);

runsim;