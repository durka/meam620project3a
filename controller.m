function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
psi = qd{qn}.euler(3);
kp = 15;
kd = 7;
% kp = 17;
% kd = 2.5;
% if norm(qd{qn}.vel_des) < 0.0001
%     kp = 5;
%     kd = 0;
% end

e = qd{qn}.pos - qd{qn}.pos_des;
edot = qd{qn}.vel - qd{qn}.vel_des;

quadAccDes = qd{qn}.acc_des - kp*e - kd*edot;

phi_des = (1/params.grav)*(quadAccDes(1)*sin(psi) - quadAccDes(2)*cos(psi));
theta_des = (1/params.grav)*(quadAccDes(1)*cos(psi) + quadAccDes(2)*sin(psi));
psi_des = qd{qn}.yaw_des;


kF = 6.11E-8;
Fhover = params.mass*params.grav;
% keyboard
% Thrust
F    = Fhover + params.mass*quadAccDes(3);
% 
% keyboard





% Moment
M    = zeros(3,1);
M(1) = (phi_des - qd{qn}.euler(1))*.4 - 0.01 * qd{qn}.omega(1);
M(2) = (theta_des - qd{qn}.euler(2))*.4 - 0.01 * qd{qn}.omega(2);
M(3) = 0;

% if norm(qd{qn}.vel_des) < 0.0001
%     M(1) = (0 - qd{qn}.euler(1))*.1 - 0.01 * qd{qn}.omega(1);
%     M(2) = (0 - qd{qn}.euler(2))*.1 - 0.01 * qd{qn}.omega(2);
%     M(3) = 0;
%     F = Fhover + params.mass*quadAccDes(3);
% end

% M(3) = ( - qd{qn}.euler(3))*0.5 - 0.01 * qd{qn}.omega(3);
% You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end