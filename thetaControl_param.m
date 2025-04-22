% Given specs & plant params
l      = 0.05;
J      = 0.01;
Ts_spec = 11;     % settling time in seconds
OS_spec = 5;     % percent overshoot
ref = 1;
% Compute zeta from overshoot
zeta = -log(OS_spec/100)/sqrt(pi^2 + log(OS_spec/100)^2);

% Compute natural frequency from settling time
omega_n = 4/(zeta*Ts_spec);

% Compute PD gains
Kd = (2*zeta*omega_n*J)/l;
Kp = (omega_n^2 * J)/l;

