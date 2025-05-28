clc;
close all;

% Simulink motor config from lab2
L = 0.0047;
R = 4.8776;
i = 48;
k_e = 0.3953;
k_m = k_e;
m = 0.0156;
r = 11.47/1000;
J = i^2 * m * r^2 / 2;

WHEEL_RADIUS = (7/2) / 100;  %# wheel radius (meters)
BASE =  11.5 / 100; % # space between centers of wheels!!! (meters)

% %# Cycle period
% T = 0.045; 

%# Reg params
KS = 15; % # linear speed
KR = 30; %# angular speed


% 
% % % % % % Task 1. Split targets
% TARGETS = [0.5, 0; 0, 0.5; -0.5, 0; 0, -0.5];
% 
% for i = 1:size(TARGETS)
%     goal_x = TARGETS(i, 1);
%     goal_y = TARGETS(i, 2);
%     ini_x = 0;
%     ini_y = 0;
%     figure;
%     axis equal;
%     hold on; grid on;
%     plot(goal_x, goal_y, 'o', LineWidth=3, DisplayName="Targets", color=[0 0 1]);
%     plot(0, 0,  'o', LineWidth=3, DisplayName="Start point", Color=[0 1 0]);
% 
%     f = readmatrix(sprintf("py/split_basic_ks120_kr160_umax90_umin25_%d", i));
%     x = f(:, 1);
%     y = f(:, 2);
%     plot(x, y, LineWidth=2, DisplayName="Robot", color=[0 0.82 1]);
%     
%     simOut = sim("lab4sim.slx", "ReturnWorkspaceOutputs", "on");
%     x_out = simOut.x.Data;
%     y_out = simOut.y.Data;
%     plot(x_out, y_out, "--" ,LineWidth=2, DisplayName="Sim", color=[0.5 0.5 0.5]);
%     legend("show", "Location","bestoutside");
% end
% 



% % % Task 2. Square traectory
TARGETS = [0.5, 0.5; -0.5, 0.5; -0.5, -0.5; 0.5, -0.5; 0.5, 0.5];
task2f = figure;
hold on; grid on;
title("");
f = readmatrix("py/base_ks120_kr160_umax90_umin25.csv");
x = f(:, 1);
y = f(:, 2);
plot(x, y, LineWidth=2, DisplayName="Robot", color=[0 0.82 1]);

ini_x = 0;
ini_y = 0;
for i = 1:size(TARGETS, 1)
    goal_x = TARGETS(i, 1);
    goal_y = TARGETS(i, 2);

    simOut = sim("lab4sim.slx", "ReturnWorkspaceOutputs", "on");
%     x_out =[x_out; simOut.x.Data];
%     y_out =[y_out; simOut.y.Data];
    x_out = simOut.x.Data;
    y_out = simOut.y.Data;

    p = plot(x_out, y_out, "--" ,LineWidth=2, color=[0.5 0.5 0.5]);  
    p.Annotation.LegendInformation.IconDisplayStyle = "off";

    ini_x = x_out(end);
    ini_y = y_out(end);
end
f = readmatrix("py/base_ks120_kr160_umax90_umin25.csv");
x = f(:, 1); y = f(:, 2);

plot(x_out, y_out, "--" ,LineWidth=2, DisplayName="Sim", color=[0.5 0.5 0.5]);
plot(TARGETS(:, 1), TARGETS(:, 2), 'o', LineWidth=3, DisplayName="Targets", color=[0 0 1]);
plot(0, 0,  'o', LineWidth=3, DisplayName="Start point", Color=[0 1 0]);

lines = findall(task2f, "Type", "Line");
lines(4).Annotation.LegendInformation.IconDisplayStyle = "on";
lines(4).DisplayName="Sim";

legend("show", "Location","bestoutside");

% 
% figure;
% hold on; grid on;
% title("Симулинк нахуй");
% ini_x = 0;
% ini_y = 0;
% for i = 1:size(TARGETS, 1)
%     goal_x = TARGETS(i, 1);
%     goal_y = TARGETS(i, 2);
% 
%     simOut = sim("lab4sim.slx", "ReturnWorkspaceOutputs", "on");
%     x_out = simOut.x.Data;
%     y_out = simOut.y.Data;
%     plot(x_out, y_out, "--" ,LineWidth=2, DisplayName="Traectory", color=[0.5 0.5 0.5]);
%     
%     ini_x = x_out(end);
%     ini_y = y_out(end);
% end
% plot(x, y, LineWidth=2, DisplayName="Traectory", color=[0 0.82 1]);
% plot(TARGETS(:, 1), TARGETS(:, 2), 'o', LineWidth=3, DisplayName="Targets", color=[0 0 1]);
% plot(0, 0,  'o', LineWidth=3, DisplayName="Start point", Color=[0 1 0]);
% 








figHandles = findobj('Type', 'figure'); % Получаем все графические окна
for i = 1:length(figHandles)
    figure(figHandles(i)); % Делаем текущей i-ю фигуру
    legend('show', 'Location','best')
    grid on;
    set(gcf, 'Position', [100, 100,     800, 600]); % Устанавливаем размеры
    % Настройка шрифтов
    set(gca, 'FontSize', 12, 'FontWeight', 'normal', 'FontName', 'Times New Roman');
    titleHandle = findobj(gca, 'Type', 'text');
    if ~isempty(titleHandle)
        set(titleHandle, 'FontSize', 12);
    end
end

