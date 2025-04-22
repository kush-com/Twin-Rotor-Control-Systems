s    = tf('s');
G_ol = 1/(m*s^2);      % plant
C_pd = Kp + Kd*s;      % PD controller
L    = C_pd * G_ol;    % open‑loop

height_param
%% Compute closed‑loop TF
T_cl = feedback(L, 1);

%% Plot root‑locus of the open‑loop and mark the closed‑loop poles
figure('Position',[300 300 600 400]);
rlocus(L); hold on;
% extract closed‑loop poles
p_cl = pole(T_cl);
% superimpose them
plot(real(p_cl), imag(p_cl), 'rx', 'MarkerSize',10, 'LineWidth',2);
title('Root Locus of L(s) with Closed‑Loop Poles');
xlabel('Real Axis'); ylabel('Imag Axis');
legend('Root Locus','Closed‑Loop Poles','Location','best');
grid on;
