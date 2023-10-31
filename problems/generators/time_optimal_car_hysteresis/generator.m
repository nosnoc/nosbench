clear all
close all
import casadi.*

LEVEL = 4; % Maybe 3 but hysteresis is _very_ difficult

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [3];
N_STAGES = [20,31];
FUEL_COST = [0,1];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for N_stg=N_STAGES
            for N_fe=N_FE
                for fuel_cost_on=FUEL_COST
                    problem_options = NosnocProblemOptions();
                    model = NosnocModel();
                    model.model_name = ['CARHYS'];
                    %%
                    problem_options.n_s = 2;
                    %% Time settings
                    problem_options.time_freezing = 1;
                    problem_options.time_freezing_hysteresis = 1;
                    problem_options.time_optimal_problem = 1;
                    % Time-freezing scaling / speed of time
                    problem_options.s_sot_max = 10;
                    problem_options.s_sot_min = 0.9;
                    problem_options.rho_sot = 1e-1;
                    problem_options.use_speed_of_time_variables = 1; 
                    problem_options.local_speed_of_time_variable = 1;
                    problem_options.stagewise_clock_constraint = 1;
                    problem_options.relax_terminal_constraint = 0;
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.lift_complementarities = lift;

                    %% Model Settings
                    problem_options.N_finite_elements = N_fe;
                    problem_options.N_stages = N_stg;
                    problem_options.T = 1;

                    %% Terminal constraint and bounds
                    q_goal = 150;
                    v_goal = 0;
                    v_max = 25;
                    u_max = 5;
                    %% Model Parameters
                    % Hystheresis parameters
                    v1 = 10;
                    v2 = 15;
                    % fual costs of turbo and nominal
                    Pn = 1;
                    Pt = 2.5;

                    %% Variable defintion
                    % states
                    q = SX.sym('q');
                    v = SX.sym('v');
                    L = SX.sym('L');
                    w = SX.sym('w');
                    t = SX.sym('t');
                    x = [q;v;L;w;t];
                    % controls
                    u= SX.sym('u');

                    model.x = x;
                    model.u = u;

                    % Bounds on x and u
                    model.lbx = -[inf;0;inf;inf;inf];
                    model.ubx = [inf;v_max;inf;inf;inf];
                    model.lbu = -u_max;
                    model.ubu = u_max;
                    %% Inital Value
                    model.x0 = zeros(5,1);
                    % u0 = 10;
                    %% PSS via Voronoi Cuts
                    z1 = [1/4;-1/4];
                    z2 = [1/4;1/4];
                    z3 = [3/4;3/4];
                    z4 = [3/4;5/4];

                    psi = (v-v1)/(v2-v1);

                    g_1 = norm([psi;w]-z1)^2;
                    g_2 = norm([psi;w]-z2)^2;
                    g_3 = norm([psi;w]-z3)^2;
                    g_4 = norm([psi;w]-z4)^2;

                    model.g_ind = [g_1;g_2;g_3;g_4];

                    % modes of the ODEs layers
                    f_A = [v;u;Pn;0;1];
                    f_B = [v;3*u;Pt;0;1];

                    a_push = 2;
                    f_push_down = [0;0;0;-a_push;0];
                    f_push_up = [0;0;0;a_push;0];

                    f_2 = f_push_down;
                    f_3 = f_push_up;
                    f_4 = 2*f_B-f_3;
                    f_1 = 2*f_A-f_2;
                    % in matrix form
                    model.F = [f_1 f_2 f_3 f_4];
                    %% objective and terminal constraint
                    model.f_q = fuel_cost_on*L;
                    % terminal constraint
                    model.g_terminal = [q-q_goal;v-v_goal];

                    %% generate mpcc
                    % TODO annoying serialization issues
                    %mpcc = NosnocMPCC(problem_options, model.dims, model);
                    problem_options.preprocess();
                    model.verify_and_backfill(problem_options);
                    model.generate_variables(problem_options);
                    model.generate_equations(problem_options);
                    filename = generate_problem_name(model, problem_options, fuel_cost_on+1)
                    save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');
                    index = index+1;
                end
            end
        end
    end
end
