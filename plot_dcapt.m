function plot_dcapt(traj)

% traj is structure with fields:
	% starts
	% goals
	% paths (contains starts, end, and intermediate switching points)
	% current (current poisitions of robots)
	% R (robot radius)
	% H (Dcapt communication distance)

tic;
    
n_T = size(traj.current,1);

range = [min([traj.starts; traj.goals],[],1)-2, max([traj.starts; traj.goals],[],1)+2];
figure(1);clf;axis([range(1) range(3) range(2) range(4)]);axis equal;
hold on; grid on; grid minor;

rx = sin(0:pi/20:2*pi);
ry = cos(0:pi/20:2*pi);

for i = 1:length(traj.paths)
    plot(traj.paths{i}(:,1),traj.paths{i}(:,2), '-.');
end

scatter(traj.starts(:,1), traj.starts(:,2),300, 's', 'markerfacecolor', 'r', 'markeredgecolor', 'k');
scatter(traj.goals(:,1), traj.goals(:,2),300, 'p', 'markerfacecolor', 'b', 'markeredgecolor', 'k');

u = unique(traj.switching(:,2:3));
v = setdiff(1:n_T, u);
circS = patch(bsxfun(@plus, traj.current(u,1), traj.H*rx)', bsxfun(@plus, traj.current(u,2), traj.H*ry)','b',...
    'edgecolor','g', 'facecolor', 'b', 'facealpha', 0.2);

circH = patch(bsxfun(@plus, traj.current(v,1), traj.H*rx)', bsxfun(@plus, traj.current(v,2), traj.H*ry)','b',...
    'edgecolor','b', 'facecolor', 'b', 'facealpha', 0.2);
circR = patch(bsxfun(@plus, traj.current(:,1), traj.R*rx)', bsxfun(@plus, traj.current(:,2), traj.R*ry)','r',...
    'edgecolor','None','facealpha', 1);
set(circR,'FaceColor', 'flat', 'FaceVertexCData',lines(n_T));

for i = 1:size(traj.switching,1)
    plot(traj.current(traj.switching(i,2:3),1), traj.current(traj.switching(i,2:3),2), 'r-');
end

xlabel('X (m)');
ylabel('Y (m)');
title(sprintf('%g s', traj.t));
drawnow
pause(traj.dt - toc);
