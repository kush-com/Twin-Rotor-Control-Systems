function test_nonlinear_pid(Kp, Ki, Kd)
% test_nonlinear_pid  Simulate nonlinear hover with given PID gains.
%   test_nonlinear_pid(Kp,Ki,Kd) runs a 10s sim and plots h(t),
%   then prints overshoot (%) and settling time (2%).

  %% Plant & sim parameters
  m   = 1.0;        % mass [kg]
  g   = 9.81;       % gravity [m/s^2]
  c_d = 0.5;        % drag coeff
  h_ref = 1.0;      % target altitude [m]
  dt = 0.01;        % time step
  t = 0:dt:100;      % time vector

  %% Init states & logs
  h = 0;          % initial height
  h_dot = 0;      % initial velocity
  e_int = 0;      % integral term   
  e_prev = h_ref; % previous error
  h_hist = zeros(size(t));

  %% Simulation loop
  for k = 1:length(t)
    % error & derivative
    e = h_ref - h;
    de = (e - e_prev)/dt;
    e_int = e_int + e*dt;

    % PID control signal
    u_pid = Kp*e + Ki*e_int + Kd*de;
    T = m*g + u_pid;    % total thrust

    % nonlinear dynamics: m*h_ddot + c_d*h_dot^2 = T - m*g
    h_ddot = (T - m*g - c_d*h_dot^2*sign(h_dot))/m;

    % Euler integration
    h_dot = h_dot + h_ddot*dt;
    h = h + h_dot*dt;

    % log
    h_hist(k) = h;
    e_prev = e;
  end

  %% Plot response
  figure('Position',[200 200 600 400]);
  plot(t, h_hist,'b','LineWidth',2); hold on;
  yline(h_ref*1.02,'r--','LineWidth',1);
  yline(h_ref*0.98,'r--','LineWidth',1);
  xlabel('Time (s)'), ylabel('Altitude h (m) or Theta (in radians)');
  title(sprintf('Nonlinear Hover PID: Kp=%.1f Ki=%.1f Kd=%.1f',Kp,Ki,Kd));
  legend('h(t)','±2% band','Location','southeast');
  grid on;

  %% Compute performance metrics
  info = stepinfo(h_hist, t, h_ref, ...
              'SettlingTimeThreshold',0.02, ...
              'RiseTimeLimits',[0.1 0.9]);
  fprintf('\nResults for [Kp Ki Kd] = [%.2f  %.2f  %.2f]\n', Kp,Ki,Kd);
  fprintf('  Overshoot   : %.2f %%\n', info.Overshoot);
  fprintf('  SettlingTime: %.2f s\n', info.SettlingTime);
  fprintf('  RiseTime    : %.2f s\n', info.RiseTime);
  fprintf('  PeakTime    : %.2f s\n', info.PeakTime);
end
