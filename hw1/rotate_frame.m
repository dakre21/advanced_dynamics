function coordinates = rotate_frame(sequence, angles)
    figure;

    coordinates = [0 0 1; 0 1 0; 1 0 0];

    R1 = @(phi)[1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R2 = @(theta)[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R3 = @(psi)[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

    for i = 1:length(sequence)
        if sequence(i) == 1
            next_frame = R1(deg2rad(angles(i))) * coordinates;
        elseif sequence(i) == 2
            next_frame = R2(deg2rad(angles(i))) * coordinates;
        else
            next_frame = R3(deg2rad(angles(i))) * coordinates;
        end

        subplot(1, 3, i);
        plot_vecs(coordinates, next_frame, sequence(i), angles(1));

        coordinates = next_frame;
    end
end

function plot_vecs(prev_frame, next_frame, rot, angle)
    axis equal;
    grid on;
    view(3);

    axis_map = {'Z', 'Y', 'X'};
    colors = {'r', 'g', 'b'}; 

    origin = [0 0 0];

    xlabel(axis_map(1));
    ylabel(axis_map(2));
    zlabel(axis_map(3));
    hold on;
    for j = 1:length(next_frame)
        v = prev_frame(j, :);
        quiver3(origin(1), origin(2), origin(3), v(1), v(2), v(3), ...
           'Color', colors{j}, 'LineWidth', 0.5, 'MaxHeadSize', 0.5);

        w = next_frame(j, :);
        quiver3(origin(1), origin(2), origin(3), w(1), w(2), w(3), ...
            'Color', colors{j}, 'LineWidth', 2, 'MaxHeadSize', 0.5);

        d = w - v;
        quiver3(v(1), v(2), v(3), d(1), d(2), d(3), 'Color', colors{j}, ...
            'LineWidth', 1, 'MaxHeadSize', 0.5, 'LineStyle', '--');
    end
       
    title(sprintf('Rotation %d about axis %d', angle, rot));
    hold off; 
    %saveas(gcf, sprintf('rotation_step_%d.png', idx));
end
