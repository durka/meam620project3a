% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** MEAM 620 QUADROTOR SIMULATION *****************
close all
clearvars;
addpath('utils')
addpath('trajectories')

% You need to implement trajhandle and controlhandle

% trajectory generator
trajhandle = @(t, qn)ccapt_traj([],[],[],[],qn,t);


% controller
controlhandle = @controller;

% real-time 
real_time = true;

% number of quadrotors

nquad = 5;
starts = rand(nquad,3)*0.5;
goals = rand(nquad,3)*1;
ccapt_traj(starts, goals, 0.3, 1, [], []);

% max time
time_tol = 13;%was 30

% parameters for simulation
params = nanoplus();

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors
for qn = 1:nquad
    % Get start and stop position
    des_start = trajhandle(0, qn);
    des_stop  = trajhandle(inf, qn);
    stop{qn}  = des_stop.pos;
    x0{qn}    = init_state( des_start.pos );
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
mystatehist = x0;

for iter = 1:max_iter

    timeint = time:tstep:time+cstep;

    tic;

    % Iterate over each quad
    for qn = 1:nquad

        % Run simulation
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn}    = xsave(end, :)';
        mystatehist{qn} = [mystatehist{qn},x{qn}];
        
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

    end
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*150)
        err = 'Ode45 Unstable';
        break;
    end

    % Check termination criteria
    term = terminate_check(x, time, stop, pos_tol, vel_tol, time_tol);
    if term
        if term == 3
            disp('Collision!');
        end
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

%% ************************* QUAD PLOTTING *************************
beep;pause;
time      = starttime; % current time
fprintf('Plotting Quads...')
for i = 1:iter
    tic;
    for qn = 1:nquad
        % Initialize quad plot
        if i == 1
            QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
            desired_state = trajhandle(time, qn);
            QP{qn}.UpdateQuadPlot(mystatehist{qn}(:,1), [desired_state.pos; desired_state.vel], time);
            h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
        end
        
        desired_state = trajhandle(time + cstep, qn);
        QP{qn}.UpdateQuadPlot(mystatehist{qn}(:,i), [desired_state.pos; desired_state.vel], time + cstep);
        set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', i, time + cstep))
    end
    time = time + cstep; % Update simulation time
    t = toc;
    if real_time && (t < cstep)
        pause(cstep - t);
    end
end

% COMPONENTS OF STATE
% 1-3 position, 4-6 velocity, 7-10 quaternion, 11-13 angular vel

% Plot the saved position and velocity of each robot
% for qn = 1:nquad
%     % Truncate saved variables
%     QP{qn}.TruncateHist();
%     % Plot position for each quad
%     h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
%     plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
%     plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
%     
%     % Plot position for each quad
%     h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : angles']);
%     [e1, e2, e3] = quat2angle(mystatehist(7:10,:)');
%     plot_state(h_pos{qn}, [e1,e2,e3]', QP{qn}.time_hist, 'pos', 'vic');
%     
%     % Plot position for each quad
%     h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : angular vel']);
%     plot_state(h_pos{qn}, mystatehist(11:13,:), QP{qn}.time_hist, 'pos', 'vic');
%     
%     % Plot velocity for each quad
%     h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
%     plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
%     plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'des');
% end
if(~isempty(err))
    error(err);
end

fprintf('finished.\n')