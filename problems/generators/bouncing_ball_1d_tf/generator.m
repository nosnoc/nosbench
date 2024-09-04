clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
INITIAL_HEIGHT = [0.04,0.2];
E = [1];
N_FE = [3,4];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for idx=1:length(INITIAL_HEIGHT)
            for e=E
                for N_fe=N_FE
                    problem_options = nosnoc.Options();
                    model = NosnocModel();
                    model.model_name = ['TIMF1D'];
                    problem_options.irk_scheme = IRKSchemes.GAUSS_LEGENDRE;
                    problem_options.n_s = 1;
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.dcs_mode = DcsMode.Step;
                    problem_options.time_freezing = 1;
                    problem_options.equidistant_control_grid = 0;
                    problem_options.impose_terminal_phyisical_time = 1;
                    problem_options.local_speed_of_time_variable = 1;
                    problem_options.lift_complementarities = lift;
                    %% model defintion
                    g = 9.81;
                    x0 = [INITIAL_HEIGHT(idx);0];

                    q = SX.sym('q',1);
                    v = SX.sym('v',1);
                    model.M = 1;
                    model.x = [q;v];
                    model.e = e;
                    model.mu_f = 0;
                    model.x0 = x0;
                    model.f_v = -g;
                    model.f_c = q;
                    %% Simulation setings
                    problem_options.T = 0.1;
                    problem_options.N_finite_elements = N_fe;
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
