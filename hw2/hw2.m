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
E_Theta = @(phi, theta)[
    1 sin(phi)*tan(theta) cos(phi)*tan(theta);
    0 cos(phi) -sin(phi); 
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)
];

R_BF = @(phi, theta, psi) [cos(psi)*cos(theta) sin(psi)*cos(theta) -sin(theta);...
    cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*cos(phi)+...
    sin(psi)*sin(theta)*cos(phi) cos(theta)*sin(phi);...
    sin(psi)*sin(phi)+cos(psi)*sin(theta)*sin(phi)...
    sin(psi)*sin(theta)*cos(psi)-cos(psi)*sin(phi) cos(theta)*cos(phi)];

attitude = zeros(N, 3);
attitude(1, 1) = phi(1);
attitude(1, 2) = theta(1);
attitude(1, 3) = psi(1);

position = zeros(N, 3);
position(1, 1) = x(1);
position(1, 2) = y(1);
position(1, 3) = z(1);

for i = 2:N
    % [\dot{\phi} \dot{\theta} \dot{psi}]^T = E(\Theta) [p q r]^T
    omega = [p(i-1) q(i-1) r(i-1)];
    euler_rates = E_Theta(attitude(i-1, 1), attitude(i-1, 2)) * omega';

    attitude(i, :) = attitude(i-1, :) + euler_rates' * dt;

    % [\dot{x} \dot{y} \dot{z}]^T = R_FB [u v w]^T
    R_FB = R_BF(attitude(i-1, 1), attitude(i-1, 2), attitude(i-1, 3))';

    vel_GB = [u(i-1) v(i-1) w(i-1)]';
    vel_GF = R_FB * vel_GB;

    position(i, :) = position(i-1, :) + vel_GF' * dt;
end

plot_euler_angles(time, phi, theta, psi, attitude);

plot_position(time, x, y, z, position);
disp('Difference between estimated and ground truth position')
error_x = norm(x - position(:, 1))
error_y = norm(y - position(:, 2))
error_z = norm(z - position(:, 3))

% (dakre) So we can see that the euler angles are very aligned however
% when performing explicit euler integration we see positional drift 
% that accumulates over time.

% Part (b) Quaternion rates + Navigation EQs
quaternion = @(phi, theta, psi) [cos(psi/2)*cos(theta/2)*...
    cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2) ...
    cos(psi/2)*cos(theta/2)*sin(phi/2)-sin(psi/2)*sin(theta/2)*cos(phi/2) ...
    cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*cos(theta/2)*sin(phi/2) ...
    sin(psi/2)*cos(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2)];

q_x_w = @(p, q, r) 0.5 * [0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];

quat_to_321 = @(quat) [
    acos((quat(1)^2-quat(2)^2-quat(3)^2+quat(4)^2)/(sqrt(1-4*(quat(2)*quat(4)-quat(1)*quat(3))^2)))*sign(2*(quat(3)*quat(4)+quat(1)*quat(2)))...
    asin(-2*(quat(2)*quat(4)-quat(1)*quat(3))) ...
    acos((quat(1)^2+quat(2)^2-quat(3)^2-quat(4)^2)/sqrt(1-4*(quat(2)*quat(4)-quat(1)*quat(3))^2))*sign(2*(quat(2)*quat(3)+quat(1)*quat(4)))
];

quat_gt = zeros(N, 4);
quat_gt(1, :) = quaternion(phi(1), theta(1), psi(1));

quat = zeros(N, 4);
quat(1, :) = quat_gt(1, :);

position2 = zeros(N, 3);
position2(1, 1) = x(1);
position2(1, 2) = y(1);
position2(1, 3) = z(1);

for i = 2:N
    quat_gt(i, :) = quaternion(phi(i), theta(i), psi(i));
    quat_dt = q_x_w(p(i), q(i), r(i)) * quat(i-1, :)';

    quat(i, :) = quat(i-1, :) + quat_dt' * dt;
    quat(i, :) = quat(i, :) / norm(quat(i, :));

    euler_angles = quat_to_321(quat(i-1, :));
    R_FB = R_BF(euler_angles(1), euler_angles(2), euler_angles(3))';

    vel_GB = [u(i-1) v(i-1) w(i-1)]';
    vel_GF = R_FB * vel_GB;

    position2(i, :) = position2(i-1, :) + vel_GF' * dt;
end

plot_quaternion(time, quat_gt, quat);

plot_position(time, x, y, z, position2);
disp('Difference between estimated and ground truth position')
error_x = norm(x - position2(:, 1))
error_y = norm(y - position2(:, 2))
error_z = norm(z - position2(:, 3))