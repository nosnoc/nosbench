clear all
close all
import casadi.*

LEVEL = 2;

CROSS_COMP_MODES = [1,3,4,7];
N_STAGES = [6,37,50,63];
N_S = [3];
N_FE = [3];
LINEAR_CONTROL = [0,1]
DCS_MODES = [DcsMode.Step, DcsMode.Stewart];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for N_stages=N_STAGES
            for n_s=N_S
                for N_fe=N_FE
                    for linear_control=LINEAR_CONTROL
                        for dcs_mode=DCS_MODES
                            problem_options = NosnocProblemOptions();
                            model = NosnocModel();
                            model.model_name = ['SMOCP'];
                            problem_options.n_s = n_s;
                            N_finite_elements = N_FE;
                            problem_options.irk_representation = 'integral';
                            problem_options.irk_scheme = IRKSchemes.RADAU_IIA;
                            problem_options.cross_comp_mode = cross_comp_mode;
                            problem_options.dcs_mode = dcs_mode;
                            problem_options.use_fesd = 1;
                            problem_options.equidistant_control_grid = 1;
                            problem_options.step_equilibration = 'heuristic_mean';  % heuristic_diff, heuristic_mean, l2_relaxed, l2_relaxed_scaled, direct, direct_homotopy, off
                            problem_options.rho_h = 1e2;
                            problem_options.lift_complementarities = lift;

                            %% model equations

                            % Variable defintion
                            x1 = SX.sym('x1');
                            x2 = SX.sym('x2');

                            v1 = SX.sym('v1');
                            v2 = SX.sym('v2');

                            % Control
                            u1 = SX.sym('u1');
                            u2 = SX.sym('u2');
                            model.u = [u1;u2];

                            if linear_control
                                v0  = [0;0];
                                x = [x1;x2;v1;v2];
                                u_max = 10;

                                % dynamics
                                f_11 = [-1+v1;0;u1;u2];
                                f_12 = [1+v1;0;u1;u2];
                                f_21 = [0;-1+v2;u1;u2];
                                f_22 = [0;1+v2;u1;u2];

                                % Objective
                                model.f_q = 1*(v1^2+v2^2)+0*(u1^2+u2^2);
                            else
                                u_max = 2;
                                v0 = [];
                                x = [x1;x2];

                                % dynamics
                                f_11 = [-1+u1;0];
                                f_12 = [1+u1;0];
                                f_21 = [0;-1+u2];
                                f_22 = [0;1+u2];

                                % Objective
                                model.f_q = u1^2+u2^2;
                            end
                            model.x0 = [2*pi/3;pi/3;v0];
                            model.x = x;
                            problem_options.T = 4;

                            problem_options.N_stages = N_stages;
                            problem_options.N_finite_elements = N_finite_elements;

                            % Switching Functions
                            p = 2; a = 0.15; a1 = 0;
                            b = -0.05; q = 3;

                            c1 = x1+a*(x2-a1)^p;
                            c2 = x2+b*x1^q;
                            model.c = {c1,c2};

                            S1 = [1;-1];
                            S2 = [1;-1];
                            model.S = {S1,S2};

                            %% Modes of the ODEs layers (for all  i = 1,...,n_sys);
                            F1 = [f_11 f_12];
                            F2 = [f_21 f_22];
                            model.F = {F1,F2};

                            % constraints
                            model.lbu  = -u_max*ones(2,1);
                            model.ubu  = u_max*ones(2,1);

                            x_target = [-pi/6;-pi/4];
                            model.g_terminal = [x(1:2)-x_target(1:2)];
                            
                            %% Solve OCP
                            % TODO annoying serialization issues
                            %mpcc = NosnocMPCC(problem_options, model.dims, model);
                            problem_options.preprocess();
                            model.verify_and_backfill(problem_options);
                            model.generate_variables(problem_options);
                            model.generate_equations(problem_options);
                            filename = generate_problem_name(model, problem_options, linear_control+1)
                            save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');
                            index = index+1;
                        end
                    end
                end
            end
        end
    end
end
