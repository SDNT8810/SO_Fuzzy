%% initializing
clear; clf; clc; % close all;

init_params
tic
Run_Timer = zeros(max_expected_size,1);

%% live Loop
while Step_Counter < max_expected_size-Window_Size
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
    if  ( Dist2Goal(Step_Counter) < 10/m2p )
        T_b = Run_Timer(Step_Counter);
        break ;
    end
end

%% Publish stop velocity commands
if (Gazebo_Sim == 1)
    velMsg.linear.x = 0;
    velMsg.angular.z = 0;
    send(velPub,velMsg);
end

toc


