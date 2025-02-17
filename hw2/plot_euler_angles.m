% MCEN 5228: Advanced Dynamics HW2 - plot euler angles
% Author: David Akre
% Date: 2/10/2025

function plot_euler_angles(time, phi, theta, psi, attitude)
    figure;

    subplot(3,1,1)
    plot(time(:, 1), rad2deg(phi), 'LineWidth', 2);
    hold on;
    plot(time(:, 1), rad2deg(attitude(:, 1)), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("\phi (deg)")
    legend("\phi_{gt}(t)", "\phi_{int}(t)")
    
    subplot(3,1,2)
    plot(time(:, 1), rad2deg(theta), 'LineWidth', 2);
    hold on;
    plot(time(:, 1), rad2deg(attitude(:, 2)), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("\theta (deg)")
    legend("\theta_{gt}(t)", "\theta_{int}(t)")

    subplot(3,1,3)
    plot(time(:, 1), rad2deg(psi), 'LineWidth', 2);
    hold on;
    plot(time(:, 1), rad2deg(attitude(:, 3)), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("\psi (deg)")
    legend("\psi_{gt}(t)", "\psi_{int}(t)")
end