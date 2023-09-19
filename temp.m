% Define symbolic variables and parameters
syms x y psi x_dot y_dot psi_dot a_x C_f C_r m l_f l_r I_z delta psi_dot_y

% Define state variables
state = [x; x_dot; y; y_dot; psi; psi_dot];
control = [a_x; delta];

% Define the right-hand side of the differential equations
x_ddot = psi * y_dot + a_x;
y_ddot = -psi * x_dot + (x_dot / y_dot) * ((C_f + C_r) / m) - (psi_dot / x_dot) * ((C_f + C_r) / m) + (C_f / m) * delta;
psi_ddot = -(x_dot / y_dot) * ((C_f * l_f + C_r * l_r) / I_z) - (psi_dot / x_dot) * ((C_f * l_f^2 + C_r * l_r^2) / I_z) + (C_f * l_f / I_z) * delta;

% Create a system of first-order differential equations
dydt = [x_dot; x_ddot; y_dot; y_ddot; psi_dot; psi_ddot];

jacobian(dydt,state)
jacobian(dydt,control)