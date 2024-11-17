clear; close all; clc;

%% initializing
init_params;
tic

%% live Loop
while 1
    Step_Counter = Step_Counter + 1;
    Run_Timer(Step_Counter) = toc;

    % Read simulation and environment parameters
    update_environment_parameters;

    % Path planner and controller algorythem
    MakingDecisions; 
    
    % Simulation and sent commands to robot
    update_dynamic_parameters;
    
    % Run RL and update network 
    learning_function;
   
    % Replot and update GUIs
    update_presentation;
    
    % End loop condition
    if ( Run_Timer(Step_Counter) > T_f )
        break ;
    end
    if  ( Dist2Goal(Step_Counter) < 0.2 ) 
        T_b = Run_Timer(Step_Counter);
        break ;
    end 
end

toc

