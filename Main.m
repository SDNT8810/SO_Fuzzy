clear;
% close all;
figure(1)
clf
clc;

%% initializing
tic
init_params

%% live Loop
while Step_Counter < max_expected_size

    Step_Counter = Step_Counter + 1;
    Run_Timer(Step_Counter) = toc;

    % Read simulation and environment parameters
    update_environment_parameters;

    % Path planner and controller algorythem
    ManageFuzzySys;
    kinodynamics;
    
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

% plot(Antcs)
% x = 0:0.001:1;
% for j = 1 : RulesNum
%     clf
%     for i = 1 : Num_MF_L2F
%         pp(i,:) = gaussmf(x , [VariMat(i,j), MeanMat(i,j)]);
%     end
% 
%     for i = 1 : Num_MF_L2F
%         hold on
%         plot(x,pp(i,:))
%     end
%     pause(0.4)
% end
% 
% 
% for j = 1 : Num_MF_L2F
%     clf
%     for i = 1 : RulesNum
%         pp(i,:) = gaussmf(x , [VariMat(j,i), MeanMat(j,i)]);
%     end
% 
%     for i = 1 : RulesNum
%         hold on
%         plot(pp(i,:))
%     end
%     pause(0.1)
% end



