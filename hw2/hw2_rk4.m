% Author: David Akre
% Date: 2/1/2025

clear all;

% Euler Angle sequence is 3-2-1 and Fixed Frame is NED

% Load and setup variables for HW2
load("quadrotor.mat");

time = quadrotor(:, 1);
x = quadrotor(:, 2);
y = quadrotor(:, 3);
z = quadrotor(:, 4);
u = quadrotor(:, 5);
v = quadrotor(:, 6);
w = quadrotor(:, 7);
phi = quadrotor(:, 8);
theta = quadrotor(:, 9);
psi = quadrotor(:, 10);
p = quadrotor(:, 11);
q = quadrotor(:, 12);
r = quadrotor(:, 13);

N = length(phi);
dt = 0.002;

% Part (a) Angular velocity in body frame to Euler Rates + Navigation EQs
function R = E_Theta(phi, theta)
    R = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
end

function R = R_BF(phi, theta, psi) 
    R = [cos(psi)*cos(theta) sin(psi)*cos(theta) -sin(theta);...
    cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*cos(phi)+...
    sin(psi)*sin(theta)*cos(phi) cos(theta)*sin(phi);...
    sin(psi)*sin(phi)+cos(psi)*sin(theta)*sin(phi)...
    sin(psi)*sin(theta)*cos(psi)-cos(psi)*sin(phi) cos(theta)*cos(phi)];
end

function xdot = f(state)
    %% [\dot{\phi} \dot{\theta} \dot{psi}]^T = E(\Theta) [p q r]^T
    omega = [state(10) state(11) state(12)];
    euler_rates = E_Theta(state(7), state(8)) * omega';

    %% [\dot{x} \dot{y} \dot{z}]^T = R_FB [u v w]^T
    R_FB = R_BF(state(7), state(8), state(9))';
    vel_GB = [state(4) state(5) state(6)]';
    vel_GF = R_FB * vel_GB;

    xdot = zeros(6, 1);
    xdot(1:3) = vel_GF;
    xdot(4:6) = euler_rates;
end

function xn = rk4_step(xk, h)
    f1 = f(xk);
    f2 = f(xk + 0.5*h*f1);
    f3 = f(xk + 0.5*h*f2);
    f4 = f(xk + h*f3);

    xn = xk(1:6)' + (h/6.0).*(f1 + 2*f2 + 2*f3 + f4);
end

state = zeros(N, 6);
state(1, 1) = x(1);
state(1, 2) = y(1);
state(1, 3) = z(1);
state(1, 4) = phi(1);
state(1, 5) = theta(1);
state(1, 6) = psi(1);

for i = 2:N
    xk = quadrotor(i-1, 2:end);
    xn = rk4_step(xk, dt);
    state(i, :) = xn;
end

plot_position(time, x, y, z, state(:, 1:3));
disp('Difference between estimated and ground truth position')
error_x = norm(x - state(:, 1))
error_y = norm(y - state(:, 2))
error_z = norm(z - state(:, 3))
