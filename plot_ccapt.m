function plot_ccapt(T, R, dt, record)

if nargin == 4
    writer = VideoWriter(record);
    open(writer);
end

n_T = size(T,1)/2;
starts = reshape(T(:,1),2,n_T)';
goals = reshape(T(:,end),2,n_T)';

range = [min([starts; goals],[],1)-2, max([starts; goals],[],1)+2];
figure(1);clf;axis([range(1) range(3) range(2) range(4)]);axis equal;
hold on; grid on; grid minor;
plot(T(1:2:end,:)', T(2:2:end,:)','b'); % plot full trajectories
scatter(starts(:,1), starts(:,2),300, 's', 'markerfacecolor', 'r', 'markeredgecolor', 'k');
scatter(goals(:,1), goals(:,2),300, 'p', 'markerfacecolor', 'b', 'markeredgecolor', 'k');

circ = @(t) viscircles(reshape(T(:,t), [2 n_T])', ones(n_T, 1)*R, 'edgecolor','g');
step = circ(1);
legend(get(gca, 'Children'), 'Robot', 'Start', 'Goal');
xlabel('X (m)');
ylabel('Y (m)');
title('Frame 1 (0 s)');

if exist('writer', 'var')
    writeVideo(writer, getframe(gcf));
end

for i = 2:size(T,2)
    tic;
    delete(step);
    step = circ(i);
    title(sprintf('Frame %d (%g s)', i, i*dt));
    drawnow;
    if exist('writer', 'var')
        writeVideo(writer, getframe(gcf));
    end
    pause(dt - toc);
end

if exist('writer', 'var')
    close(writer);
end
