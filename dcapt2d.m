function dcapt2d(starts, goals, R, H, tf, dt, plotting)

close all
if nargin < 7
    plotting = false;
end

[starts,goals] = shift_points(starts,goals,R);


N = size(starts, 1);
n = size(starts, 2);

if size(goals, 1) ~= N
    error('Different numbers of start and goal points!');
end

if size(goals, 2) ~= n
    error('Different dimensions in start and goal space!');
end

traj.starts = starts;
traj.goals = goals;
traj.R = R;
traj.H = H;
traj.current = starts;
traj.paths = cell(1,N);
for i=1:N
    traj.paths{i} = [starts(i,:); goals(i,:)];
end
traj.switching = zeros(0,3);
traj.dt = dt;
traj.t = 0;

v = (goals - starts)/tf; 

if ischar(plotting)
    writer = VideoWriter(plotting);
    open(writer);
end

current = starts;
for tc = 0:dt:(tf-dt)
    dists = squareform(pdist(current));
    % Do DCAPT
    traj.switching(traj.switching(:,1) < (tc-3*dt), :) = [];
    for j = 1:N
      for i = 1:N
        if i == j
          continue
        end
        if dists(i,j) <= H 
          uw = dot((current(i,:) - current(j,:)),(goals(i,:) - goals(j,:)));
          if uw < 0
            goal1 = goals(i,:);
            goal2 = goals(j,:);
            goals(i,:) = goal2;
            goals(j,:) = goal1;

            v(i,:) = (goals(i,:) - current(i,:))/(tf - tc);
            v(j,:) = (goals(j,:) - current(j,:))/(tf - tc);
            
            traj.paths{i}(end,:) = current(i,:);
            traj.paths{i}(end+1,:) = goals(i,:);
            traj.paths{j}(end,:) = current(j,:);
            traj.paths{j}(end+1,:) = goals(j,:);
            
            traj.switching(end+1,:) = [tc i j];
            
            fprintf('switching %d and %d \n',i,j)
          end
          
        end
      end
    end
    current = current + v*dt;
    traj.current = current;
    traj.t = tc;
    
    if plotting
        plot_dcapt(traj);
        if ischar(plotting)
            writeVideo(writer, getframe(gcf));
        end
    end
end
 
if ischar(plotting)
    close(writer);
end
      
    
    % Break when near goals
    
end