function plot_ccapt(T, dt)

n_T = size(T,1)/2;
starts = reshape(T(:,1),2,n_T)';
goals = reshape(T(:,end),2,n_T)';

range = [min([starts; goals],[],1)-5, max([starts; goals],[],1)+5];
figure(1);clf;axis([range(1) range(3) range(2) range(4)]);
hold on;
plot(T(1:2:end,:)', T(2:2:end,:)','b'); % plot full trajectories
scatter(starts(:,1), starts(:,2),300, 's', 'markerfacecolor', 'r', 'markeredgecolor', 'k');
scatter(goals(:,1), goals(:,2),300, 'p', 'markerfacecolor', 'b', 'markeredgecolor', 'k');

step = scatter(T(1:2:end,1)', T(2:2:end,1)',200, 'o', 'markerfacecolor', 'g', 'markeredgecolor', 'none');

for i = 2:size(T,2)
    set(step,'xdata', T(1:2:end,i)', 'ydata', T(2:2:end,i)');
    drawnow;pause(dt);
end
