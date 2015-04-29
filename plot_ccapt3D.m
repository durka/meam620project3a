function plot_ccapt3D(T, R, dt)

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
scatter3(starts(:,1), starts(:,2), starts(:,3),150, 's', 'markerfacecolor', 'r', 'markeredgecolor', 'k');
scatter3(goals(:,1), goals(:,2), goals(:,3),150, 'p', 'markerfacecolor', 'b', 'markeredgecolor', 'k');

%step = scatter(T(1:2:end,1)', T(2:2:end,1)',200, 'o', 'markerfacecolor', 'g', 'markeredgecolor', 'none');
step = cell(n_T, 1);
[sx, sy, sz] = sphere(5);
for i=1:n_T
    x = sx*R + T( (i-1)*3+1 , 1);
    y = sy*R + T( (i-1)*3+2 , 1);
    z = sz*R + T( (i-1)*3+3 , 1);
    step{i} = surf(x, y, z, 'facecolor','g', 'edgecolor','none', 'facealpha',0.5);
end

for i = 2:size(T,2)
    tic;
    for j=1:n_T
        x = sx*R + T( (j-1)*3+1 , i);
        y = sy*R + T( (j-1)*3+2 , i);
        z = sz*R + T( (j-1)*3+3 , i);
        set(step{j}, 'xdata', x, 'ydata', y, 'zdata', z);
    end
    drawnow;
    t = toc;
    if t < dt;
        pause(dt-t);
    end
end
