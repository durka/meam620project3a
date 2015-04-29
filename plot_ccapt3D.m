function plot_ccapt3D(T, dt)

n_T = size(T,1)/3;
starts = reshape(T(:,1),3,n_T)';
goals = reshape(T(:,end),3,n_T)';

range = [min([starts; goals],[],1)-5, max([starts; goals],[],1)+5];
figure(1);clf;
axis vis3d;
grid on;
view(45, 45);
axis([range(1) range(4) range(2) range(5) range(3) range(6) 0 1]);
hold on;
plot3(T(1:3:end,:)', T(2:3:end,:)',T(3:3:end,:)','b'); % plot full trajectories
scatter3(starts(:,1), starts(:,2), starts(:,3),300, 's', 'markerfacecolor', 'r', 'markeredgecolor', 'k');
scatter3(goals(:,1), goals(:,2), goals(:,3),300, 'p', 'markerfacecolor', 'b', 'markeredgecolor', 'k');

step = scatter(T(1:2:end,1)', T(2:2:end,1)',200, 'o', 'markerfacecolor', 'g', 'markeredgecolor', 'none');

for i = 2:size(T,2)
    set(step,'xdata', T(1:3:end,i)', 'ydata', T(2:3:end,i)','zdata', T(3:3:end,i)');
    drawnow;pause(dt);
end
