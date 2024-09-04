clear all
close all
import casadi.*

LEVEL = 3;

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [4];
N_STAGES = [40, 43];
OMEGAS = [-2*pi,-3*pi];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for N_stg=N_STAGES
            for N_fe=N_FE
                for idx=1:length(OMEGAS)
                    problem_options = nosnoc.Options();
                    model = NosnocModel();
                    

                    %% Model Settings
                    problem_options.N_finite_elements = N_fe;
                    problem_options.N_stages = N_stg;

                    %%
                    problem_options.n_s = 2;
                    problem_options.use_speed_of_time_variables =  1; 
                    problem_options.local_speed_of_time_variable = 1;  
                    problem_options.stagewise_clock_constraint = 1;
                    problem_options.time_freezing = 1;
                    problem_options.lift_complementarities = lift;
                    problem_options.cross_comp_mode = cross_comp_mode;
                    %% Generate Model
                    % angulary velocity of reference
                    omega = OMEGAS(idx);
                    N_periods = 2;
                    problem_options.T = N_periods*(2*pi/abs(omega));
                    model = elastic_ball_in_box_model(omega);
                    model.model_name = ['TFBIB'];

                    %% generate mpcc
                    % TODO annoying serialization issues
                    %mpcc = NosnocMPCC(problem_options, model.dims, model);
                    problem_options.preprocess();
                    model.verify_and_backfill(problem_options);
                    model.generate_variables(problem_options);
                    model.generate_equations(problem_options);
                    filename = generate_problem_name(model, problem_options, idx)
                    save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');
                    index = index+1;
                end
            end
        end
    end
end
