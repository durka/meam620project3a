% do the thing
starts = [1 2; 3 4];
goals = [10 12; 30 40];
T = ccapt(starts, goals, 1, 1, .1);

clf;
plot(T(1,:), T(2,:), 'r', T(3,:), T(4,:), 'g'); % plot full trajectories
hold on;
for i=1:size(T,2) % animate!
    plot(T(1,i), T(2,i), 'r.', T(3,i), T(4,i), 'g.', 'markersize',20)
    drawnow
end
