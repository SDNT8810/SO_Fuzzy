%% initializing
clear; clf; clc; % close all;

init_params
tic
Run_Timer = zeros(max_expected_size,1);

%% live Loop
while Step_Counter < max_expected_size-Window_Size
% while Step_Counter < 220
    Step_Counter = Step_Counter + 1;
    Run_Timer(Step_Counter) = toc;
    
    % Read simulation and environment parameters
    update_environment_parameters;

    % Path planner and controller algorythem
    ManageFuzzySys;

    % Simulation and sent commands to robot
    kinodynamics;

    % Run RL and update network
    learning_proccess;

    % Replot and update GUIs
    update_presentation;

    % End loop condition
    if ( Run_Timer(Step_Counter) > T_f )
        break ;
    end
    if  ( Dist2Goal(Step_Counter) < Params.R )
        T_b = Run_Timer(Step_Counter);
        disp('(REAL World:) Goal !!!')
        break ;
    end
    if ((1) && (min(Params.Lidar_Range - Points360) < Robot.R))
        disp('(REAL World:) hit the wall !!!');
    end
end

%% plot path
subplot(2,3,[1,2,4,5])
hold on
plot_path = plot(m2p * (X(1,1:h)),m2p * (X(3,1:h)), 'LineWidth', 2, 'LineStyle', '--', 'color', 'black');
hLegend = legend([startPoint, targetPoint, plot_path], {'Start Point', 'Target', 'Path'}, 'FontSize', 24, 'Location', 'best');
set(hLegend, 'FontName', 'Times New Roman');
delete(lidar_line);

%% Publish stop velocity commands
if (Gazebo_Sim == 1)
    velMsg.linear.x = 0;
    velMsg.angular.z = 0;
    send(velPub,velMsg);
end

toc

%% Save Data
% save FuzzySystemLongMemory MeanMat VariMat W RulesNum age_of_Rulls

%% End

