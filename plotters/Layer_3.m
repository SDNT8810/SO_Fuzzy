% Create Figure
clc
figure(2);
clf
hold on;

% Add functions directory to path
addpath('../functions');

% Example Data
Node_2 = Node_2_Output; % Replace with your actual Node_2 data
Node_3 = Node_3_Output; % Replace with your actual Node_3 data
Other_Inputs = rand(1,1); % Replace with your actual Other_Inputs data

antecedents = [Antcs;rand(1,1)];

% Group Data
group_labels = {'Node 2', 'Goal Distance', 'Node 3'};
data = {Node_2, Other_Inputs, Node_3};


% Parameters
bar_width = 0.8; % Width of each filled box
colors = lines(length(data)); % Generate distinct colors
x_offset = 0; % X-offset for the first group
brace_y = -0.05; % Y-position for braces (below the x-axis)
brace_height = 0.03; % Height of the brace curve
x_positions = []; % Store x positions for labeling

% Plot Data Using `fill`
for i = 1:length(data)
    x_values = x_offset + (1:length(data{i}));
    for j = 1:length(data{i})
        % Define rectangle coordinates for `fill`
        x_rect = [x_values(j) - bar_width/2, x_values(j) + bar_width/2, ...
                  x_values(j) + bar_width/2, x_values(j) - bar_width/2];
        y_rect = [0, 0, data{i}(j), data{i}(j)];
        fill(x_rect, y_rect, colors(i,:), 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    end
    x_positions = [x_positions; mean(x_values)]; % Store x position for braces
    x_offset = x_offset + length(data{i}) + 1; % Adjust x-offset for the next group
end

% Add Large Horizontal Braces Below Groups
for i = 1:length(data)
    % Define brace curve
    x_start = x_positions(i) - length(data{i}) / 2 - 0.5;
    x_end = x_positions(i) + length(data{i}) / 2 + 0.5;
    x_brace = linspace(x_start, x_end, 100);
    y_brace = brace_y - brace_height * sin((x_brace - x_start) * pi / (x_end - x_start));

    % Plot the brace
    plot(x_brace, y_brace, 'k-', 'LineWidth', 4.5);

    % Add group label below the brace
    text(mean(x_brace), brace_y - 0.06, group_labels{i}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontSize', FontSize);
end

%% Antecedents

% Create a new set of axes
% ax2 = axes('Position', [0.1, 0.1, 0.8, 0.8]); % Adjust the position as needed
% ax2.YDir = 'reverse'; % Reverse the Y-direction
% ax2.XAxisLocation = 'top'; % Move the X-axis to the top
% hold on;

% Plot your data upside-down
% Antecedents = fill((1:2:x_offset), antecedents, [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
Antecedents = fill([1:2:x_offset, fliplr(1:2:x_offset)], [antecedents', ones(size(antecedents'))], ...
    [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
% Antecedents = plot((1:2:x_offset), antecedents, 'LineWidth', 4.5);

% Keep the original axes for other plots
% original_ax = gca; % Store the current axes

%% Formatting
grid on;
% axis([0 360 0 1.1]); % Adjust axis limits if needed
% ylim([brace_y - 0.1, max(cellfun(@max, data)) + 0.1]);
ylim([brace_y - 0.1, 1.1]);
xlim([0, x_offset]);
% hLegend = legend([blueLine, Filled, Gaussian], {'Input', 'Output', 'Weighting'}, 'FontSize', FontSize, 'Location', 'best');
% set(hLegend, 'FontName', 'Times New Roman');
hold off;

% title('Gaussian filter for weighting based on robot heading', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
xlabel('Inputs of layer 3', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');
ylabel('Values', 'FontSize', FontSize, 'FontWeight', 'bold', 'FontName', 'Times New Roman', 'Color', 'black');

% Adjust the axes to ensure ruler-like appearance
ax = gca; % Get current axes
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridColor = 'black'; % Grid color
ax.GridAlpha = 1.0; % Grid transparency
ax.LineWidth = 1.5; % Grid line thickness
ax.GridLineStyle = '-'; % Dashed grid lines
ax.GridAlpha = 0.01; % Transparency of grid lines

% Enable axis ticks and labels
axis on;
set(ax, 'TickDir', 'out', 'FontSize', FontSize, 'FontWeight', 'bold'); % Customize tick appearance

box on

% Create a secondary x-axis at the top
ax_top = axes('Position', ax.Position, 'XAxisLocation', 'top', 'YAxisLocation', 'right', ...
    'Color', 'none', 'XColor', 'black', 'YColor', 'black', 'Box', 'on');

set(ax_top, 'TickDir', 'out', 'FontSize', FontSize, 'FontWeight', 'bold'); % Customize tick appearance

% ax_top.YAxisLocation = 'right';
% ax_top.XAxisLocation = 'top';
ax_top.XGrid = 'on';
ax_top.YGrid = 'on';
ax_top.GridColor = 'black'; % Grid color
ax_top.GridAlpha = 1.0; % Grid transparency
ax_top.LineWidth = 1.5; % Grid line thickness
% ax_top.GridLineStyle = '-'; % Dashed grid lines
ax_top.GridAlpha = 0.01; % Transparency of grid lines
ax_top.YDir = 'reverse'; % Reverse the direction of the secondary Y-axis


% Set the x-axis limits and labels
xlim(ax_top, [0, size(antecedents, 1)]);
ylim(ax_top,[-0.1, 1.5]);
xlabel(ax_top, 'Rull number', 'FontSize', FontSize, 'FontWeight', 'bold', ...
    'FontName', 'Times New Roman', 'Color', 'black');

box off
axis on


