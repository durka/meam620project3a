function dcapt2D(starts, goals, R, H, vmax, dt, plotting)

traj.starts = starts;
traj.goals = goals;
traj.R = R;
traj.H = H;
traj.current = starts;
n_bots = size(starts,1);

working = true;



N = size(starts, 1);
n = size(starts, 2);

if size(goals, 1) ~= N
    error('Different numbers of start and goal points!');
end

if size(goals, 2) ~= n
    error('Different dimensions in start and goal space!');
end

v = (goals - starts)/tf; 

current = starts;
for tc = 0:dt:tf
    dists = squareform(pdist(current));
    % Do DCAPT
    for j = 1:n_bots
      for i = 1:n_bots
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
          end
          
        end
      end
    end
    current = current + v.*dt;
    traj.current = current;
    plot_
end
 
 
      
    
    % Break when near goals
    
end