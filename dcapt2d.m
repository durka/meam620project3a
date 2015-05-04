function dcapt2D(starts, goals, R, H, vmax, dt, plotting)

traj.starts = starts;
traj.goals = goals;
traj.R = R;
traj.H = H;
traj.current = starts;
n_bots = size(starts,1);

working = true;

% initialize proximity and update set
prox_set{n_bots} = [];

dists = squareform(pdist(starts));
dists(eye(n_bots)) = max(dists(:)) + 10;

for i = 1:n_bots
    prox_set{i} = find(dists(i,:)) <= H;
end

update_set = prox_set;

while working
    
    % Do DCAPT
    
    % Break when near goals
    
end