
clear all
close all
import casadi.*

LEVEL = 4;

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [3];
N_STAGES = [22,25,31];
OBSTACLE_RADIUS = [0.5,1,1.5];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stg=N_STAGES
        for N_fe=N_FE
            for idx=1:length(OBSTACLE_RADIUS)
                problem_options = nosnoc.Options();
                model = nosnoc.model.Cls();
                model_name = ['DSCOB'];

                problem_options.rk_scheme = RKSchemes.RADAU_IIA;
                problem_options.n_s = 2;  % number of stages in RK methods

                problem_options.use_fesd = 1;
                problem_options.time_freezing = 1;
                problem_options.stabilizing_q_dynamics = 1;

                problem_options.cross_comp_mode = cross_comp_mode;
                problem_options.dcs_mode = DcsMode.Heaviside;
                % enforce inequality at finite elements.
                problem_options.g_path_at_fe = 1;
                problem_options.g_path_at_stg = 1;
                problem_options.a_n = 10;

                %% discretizatioon
                T = 5;

                %% model parameters
                m1 = 2;
                m2 = 1;
                r1 = 0.3;
                r2 = 0.2;
                r_ob = OBSTACLE_RADIUS(idx);

                ubx = [10; 10;10; 10; 5; 5; 5; 5];
                lbx = -ubx;

                q10 = [-2; 0];
                q20 = [2;0];
                q_target1 = q20;
                q_target2 = q10;
                v10 = [0;0];
                v20 = [0;0];
                x0 = [q10;q20;v10;v20];

                u_ref = [0;0];
                x_ref = [q_target1;q_target2;zeros(4,1)];

                Q = diag([10;10;10;10;0*ones(4,1)]);
                R = diag([0.1 0.1]);
                Q_terminal = 100*Q;

                ubu = [30;30];
                lbu = -ubu;

                %% Symbolic variables and bounds
                q = SX.sym('q',4);
                v = SX.sym('v',4);
                u = SX.sym('u',2);

                q1 = q(1:2);
                q2 = q(3:4);
                v1 = v(1:2);
                v2 = v(3:4);

                x = [q;v];
                problem_options.T = T;
                problem_options.N_stages = N_stg;
                problem_options.N_finite_elements = N_fe;
                model.x = x;
                model.u = u;
                model.e = 0;
                model.mu = 0.0;
                model.x0 = x0;
                model.dims.n_dim_contact = 2;

                cv = 2;
                eps = 1e-1;
                f_drag = cv*[v1/norm(v1+eps);v2/norm(v2+eps)];

                model.M = diag([m1;m1;m2;m2]); % inertia/mass matrix;
                model.f_v = [u;...
                    zeros(2,1)]-f_drag;

                %% gap functions
                model.f_c = [norm(q1-q2)^2-(r1+r2)^2];

                %% obstacle
                q_ob = [0;0];
                g_path = -[(q1(1)-q_ob(1))^2+(q1(2)-q_ob(2))^2-(r_ob+r1)^2;...
                    (q2(1)-q_ob(1))^2+(q2(2)-q_ob(2))^2-(r_ob+r2)^2];
                model.g_path = g_path;

                %% box constraints on controls and states
                model.lbu = lbu;
                model.ubu = ubu;
                model.lbx = lbx;
                model.ubx = ubx;
                %%  Objective
                model.f_q = (x-x_ref)'*Q*(x-x_ref)+ u'*R*u;
                model.f_q_T = (x-x_ref)'*Q_terminal*(x-x_ref);
                %% Generate problem
                filename = generate_problem_name(model_name, model, problem_options, 1);
                %% Save problem
                discrete_time_problem = generate_problem(model, problem_options);
                json = jsonencode(discrete_time_problem, "ConvertInfAndNaN", false, "PrettyPrint", true);
                casadi_json = discrete_time_problem.to_casadi_json();
                fid = fopen(['../../vdx/', char(filename), '.json'], 'w');
                fprintf(fid, '%s', json);
                fclose(fid);
                fid = fopen(['../../casadi/', char(filename), '.json'], 'w');
                fprintf(fid, '%s', casadi_json);
                fclose(fid);
                index = index+1;
            end
        end
    end
end
