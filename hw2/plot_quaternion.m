% MCEN 5228: Advanced Dynamics HW2 - plot quaternion
% Author: David Akre
% Date: 2/10/2025

function plot_quaternion(time, quaternion, attitude)
    figure;

    subplot(4,1,1)
    plot(time(:, 1), quaternion(:, 1), 'LineWidth', 2);
    hold on;
    plot(time(:, 1), attitude(:, 1), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("q0")
    legend("q0_{gt}(t)", "q0_{int}(t)")

    subplot(4,1,2)
    plot(time(:, 1), quaternion(:, 2), 'LineWidth', 2);
    hold on;
    plot(time(:, 1), attitude(:, 2), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("q1")
    legend("q1_{gt}(t)", "q1_{int}(t)")

    subplot(4,1,3)
    plot(time(:, 1), quaternion(:, 3), 'LineWidth', 2);
    hold on;
    plot(time(:, 1), attitude(:, 3), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("q2")
    legend("q2_{gt}(t)", "q2_{int}(t)")

    subplot(4,1,4)
    plot(time(:, 1), quaternion(:, 4), 'LineWidth', 2);
    hold on;
    plot(time(:, 1), attitude(:, 4), 'r', 'LineWidth', 2, 'LineStyle', '--');
    hold off;
    xlabel("Time (s)")
    ylabel("q3")
    legend("q3_{gt}(t)", "q3_{int}(t)")
end