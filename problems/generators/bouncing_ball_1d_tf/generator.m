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
    for idx=1:length(INITIAL_HEIGHT)
        for e=E
            for N_fe=N_FE
                problem_options = nosnoc.Options();
                model = nosnoc.model.Cls();
                model_name = ['TIMF1D'];
                problem_options.rk_scheme = RKSchemes.GAUSS_LEGENDRE;
                problem_options.n_s = 1;
                problem_options.cross_comp_mode = cross_comp_mode;
                problem_options.dcs_mode = DcsMode.Heaviside;
                problem_options.time_freezing = 1;
                problem_options.equidistant_control_grid = 0;
                problem_options.impose_terminal_phyisical_time = 1;
                problem_options.local_speed_of_time_variable = 1;

                %% model defintion
                g = 9.81;
                x0 = [INITIAL_HEIGHT(idx);0];

                q = SX.sym('q',1);
                v = SX.sym('v',1);
                model.M = 1;
                model.x = [q;v];
                model.e = e;
                model.mu = 0;
                model.x0 = x0;
                model.f_v = -g;
                model.f_c = q;
                %% Simulation setings
                problem_options.T = 0.1;
                problem_options.N_finite_elements = N_fe;
                %% Generate problem
                filename = generate_problem_name(model_name, model, problem_options, idx);
                %% Save problem
                discrete_time_problem = generate_problem(filename, model, problem_options);
                index = index+1;
            end
        end
    end
end
