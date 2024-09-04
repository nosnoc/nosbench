clear all
close all
import casadi.*

LEVEL = 2;

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [3];
N_STAGES = [10,12,35,40];
DCS_MODES = [DcsMode.Step, DcsMode.Stewart];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false,true]
        for N_stg=N_STAGES
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = NosnocModel();
                    model.model_name = ['CARTIM'];
                    %%
                    problem_options.n_s = 2;
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.use_fesd = 1;
                    problem_options.use_speed_of_time_variables = 1;
                    problem_options.dcs_mode = dcs_mode;
                    problem_options.lift_complementarities = lift;

                    %% Time settings
                    problem_options.time_optimal_problem = 1;
                    %% Model - define all problem functions and
                    % Discretization parameters
                    problem_options.N_stages = N_stg; % number of control intervals
                    problem_options.N_finite_elements = N_fe; % number of finite element on every control intevral (optionally a vector might be passed)
                    problem_options.T = 1;    % Time horizon

                    % Symbolic variables and bounds
                    q = SX.sym('q'); % position
                    v = SX.sym('v'); % velocity
                    model.x = [q;v]; % add all important data to the struct model,
                    model.x0 = [0;0]; % inital value

                    v_max = 20; % maximal velocity
                    model.lbx = [-inf;-v_max];
                    model.ubx = [inf;v_max];

                    % control
                    u = SX.sym('u');
                    model.u = u;
                    % bounds
                    u_max = 5;
                    model.lbu = -u_max;
                    model.ubu = u_max;

                    % Dyanmics and the regions
                    f_1 = [v;u]; % mode 1 - nominal
                    f_2 = [v;3*u]; % mode 2 - turbo
                    model.F = [f_1 f_2];
                    % Constraint for the regions
                    v_trash_hold = 10;
                    model.c = v-v_trash_hold;
                    model.S = [-1;1];
                    %% terminal constraint
                    q_goal = 200;
                    v_goal = 0;
                    % Add terminal constraint, if no upper and lower bound are provided, they are set to zero
                    model.g_terminal = [q-q_goal;v-v_goal];

                    %% generate mpcc
                    % TODO annoying serialization issues
                    %mpcc = NosnocMPCC(problem_options, model.dims, model);
                    problem_options.preprocess();
                    model.verify_and_backfill(problem_options);
                    model.generate_variables(problem_options);
                    model.generate_equations(problem_options);
                    filename = generate_problem_name(model, problem_options, 1)
                    save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');
                    index = index+1;
                end
            end
        end
    end
end
