clearvars;
% starts = [-2 -1; -1 -1; 0 -1; 1 -1; 2 -1;
%           -2 4; -1 4; 0 4; 1 4; 2 4;
%           -15 1.5];
% goals = [-1 1; 0 1; 1 1; 2 1; 3 1;
%          -1 2; 0 2; 1 2; 2 2; 3 2;
%          5 1.5];

starts1 = [-5 2; 5 2;
          -5 -3; 5 -3;
          -5 1; 5 1];
goals1 = [7 -1; -7, -1;
         -5 0; 5 0;
         -5 2; 5 2];
R = 0.2;
H = 2*sqrt(2)*R;
tf = 2;
dt = 0.05;

% dcapt2d(starts1, goals1, R, H, tf, dt, true)

%%
starts2 = [1:10; ones(1,10)]';    
starts2(end+1,:) = [35,10];
starts2(end+1,:) = [-10,10];

goals2 = [1:10; (1:10)+5]';
goals2(end+1,:) = [-5,10];
goals2(end+1,:) = [30,10];

% dcapt2d(starts2, goals2, R, H, tf, dt, true)
% plot_ccapt(ccapt(starts2, goals2, R, 10, dt), R,dt)

%%
% starts3 = [-10 0; 10 0];    
% goals3 = [10 0; -10 0];

d = 0:0.25*pi:2*pi;
d(end)=[];
Rc = 3;
starts3 = [Rc*sin(d)' Rc*cos(d)'];
goals3 = [Rc*sin(d+pi)' Rc*cos(d+pi)'];

tf = 5;

dcapt2d(starts3, goals3, R, H, tf, dt, true)
