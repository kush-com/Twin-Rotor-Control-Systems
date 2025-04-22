clc; clear variables; 
global u_in

% Load params
run('thetaControl_param');   % defines l, J, Kp, Kd, ref

% Time setup
t_final    = 50;
dt         = 0.01;
time_vec   = 0:dt:t_final;   % 1×N
N          = numel(time_vec);

% State logs
state_log  = zeros(N,2);     % [theta, theta_dot]
time_log   = time_vec;       % copy for clarity

% Initials
state      = [0 0];
e_prev     = ref - state(1);

%% Simulation
for k = 1:N
    % current time
    t_now = time_vec(k);
    
    % current state
    theta     = state(1);
    theta_dot = state(2);
    
    % PD control
    e      = ref - theta;
    e_der  = (e - e_prev)/dt;
    u_in   = Kp*e + Kd*e_der;
    e_prev = e;
    
    % Integrate one step (over 3 points)
    tspan = [t_now, t_now+dt, t_now+2*dt];
    [~, Y] = ode45(@thetaControl_ode, tspan, state);
    
    % log the *midpoint* result at t_now+dt
    state       = Y(2,:);      
    state_log(k,:) = state;
end

theta_hist     = state_log(:,1);
theta_dot_hist = state_log(:,2);
fprintf('Suggested Gains → Kp = %.3f,  Kd = %.3f\n', Kp, Kd);

%% Plot
figure;
plot(time_log, theta_hist, 'b-', 'LineWidth',2); hold on;
plot(time_log, theta_dot_hist, 'r--','LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('States');
legend('\theta','\thetȧ');
title('Theta & Theta-dot vs Time');
