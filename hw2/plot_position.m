% MCEN 5228: Advanced Dynamics HW2 - plot position
% Author: David Akre
% Date: 2/10/2025

function plot_position(time, x, y, z, position)
    figure;

    subplot(3,1,1)
    plot(time(:, 1), x, 'LineWidth', 2);
    hold on;
    plot(time(:, 1), position(:, 1), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("x (m)")
    legend("x_{gt}(t)", "x_{int}(t)")
    
    subplot(3,1,2)
    plot(time(:, 1), y, 'LineWidth', 2);
    hold on;
    plot(time(:, 1), position(:, 2), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("y (m)")
    legend("y_{gt}(t)", "y_{int}(t)")

    subplot(3,1,3)
    plot(time(:, 1), z, 'LineWidth', 2);
    hold on;
    plot(time(:, 1), position(:, 3), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("z (m)")
    legend("z_{gt}(t)", "z_{int}(t)")
end