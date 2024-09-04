clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [5];
N_S = [2];
CONDS = {4,[0;0.2000;0;0.6000;0;1.0000;-2.2000;1.0000;0;0;0;0;0;0;20.0000;0;0];
    4,[0.0000;0.2000;0.0000;0.6000;0.0000;1.0000;-0.4500;0.3820;0.0000;0.0000;0.0000;0.0000;0.0000;0.0000;4.9999;-3.4335;0.350];
    3,[0.0000;0.2000;0.0000;0.6000;-0.7000;0.4823;0.0000;0.0000;0.0000;0.0000;10.0000;-1.4715;0.1500];
    5,[0.0000;0.2000;0.0000;0.6000;0.0000;1.0000;0.0000;1.4000;-1.4500;1.3853;0.0000;0.0000;0.0000;0.0000;0.0000;0.0000;0.0000;0.0000;15.0000;-0.4905;0.0500];};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for N_fe=N_FE
            for n_s=N_S
                for idx=1:length(CONDS)
                    problem_options = nosnoc.Options();
                    model = NosnocModel();
                    model.model_name = ['TFPOB'];
                    

                    %% Discretization Settings
                    problem_options.N_finite_elements = N_fe;
                    problem_options.n_s = n_s;
                    problem_options.T = 0.1;

                    %% Problem_opts
                    problem_options.irk_scheme = IRKSchemes.RADAU_IIA;
                    problem_options.time_freezing = 1;
                    problem_options.impose_terminal_phyisical_time = 1;
                    problem_options.local_speed_of_time_variable = 1;
                    problem_options.equidistant_control_grid = 0;
                    problem_options.stagewise_clock_constraint = 0;
                    problem_options.pss_lift_step_functions = 0;
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.lift_complementarities = lift;

                    %% Model
                    % dimension
                    model.a_n = 100;
                    model.dims.n_dim_contact = 2;
                    n_balls = CONDS{idx,1};
                    n_q = n_balls*2; % number of positions
                                     % parameters
                    m = 1;
                    M = m*eye(n_q);
                    g = 9.81;
                    mu = 0;
                    e = 0;
                    % Differential state
                    q = SX.sym('q',n_q);
                    v = SX.sym('v',n_q);
                    x = [q;v];
                    r = 0.2*ones(n_balls,1);
                    %% switching functions
                    q_pos = {};
                    for ii = 0:n_balls-1
                        q_pos{ii+1} = q(2*(ii+1)-1:2*(ii+1));
                    end
                    f_c = [];
                    for ii = 1:n_balls-1
                        for jj = ii:n_balls
                            if ii~=jj
                                f_c = [f_c;norm(q_pos{ii}-q_pos{jj})^2-(r(ii)+r(jj))^2];
                            end
                        end
                    end
                    % walls
                    wall_up = 2;
                    wall_down = 0;
                    wall_left = -3;
                    wall_right = 3;

                    for ii = 1:n_balls
                        q_temp = q_pos{ii};
                        f_walls_ii = [q_temp(2)-(wall_down+r(ii))];
                        f_c = [f_c;f_walls_ii];
                    end
                    %%
                    % all forces without constraints (free flight dynamics)
                    g = 9.81;
                    f_g = [0;-g];
                    f_g = repmat(f_g,n_balls,1);
                    f_v = zeros(n_q,1)+f_g;

                    x0 = CONDS{idx,2};
                    %% populate model
                    model.e = 0;
                    model.q = q;
                    model.v = v;
                    model.x = x;
                    model.mu_f = mu;
                    model.M = M;
                    model.f_v = f_v;
                    model.x0 = x0(1:end-1);
                    model.f_c = f_c;
                    
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
