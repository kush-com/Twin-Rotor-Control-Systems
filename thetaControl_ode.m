% second_order_ODE.m
function xdot = thetaControl_ode(t, x)

    % Global variable for control input
    global u_in;

    % Load system parameters
    theta_Param  % Make sure this file exists and defines l and J

    % Define the system dynamics (second-order ODE)
    xdot = [
        x(2);  % First equation: theta_dot (velocity)
        (l / J) * u_in;  % Second equation: theta double dot (acceleration)
    ];

end
