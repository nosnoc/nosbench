clear all
close all
import casadi.*

LEVEL = 2;

CROSS_COMP_MODES = [7];
N_STAGES = [10, 50];
N_S = [3];
N_FE = [2];

T = 10;
x_target = [0;-4];

% An optimal control problem from:
% Finite Elements with Switch Detection for Numerical Optimal Control of 
% Projected Dynamical Systems
% Anton Pozharskiy, Armin Nurkanovic, Moritz Diehl

index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stages=N_STAGES
        for n_s=N_S
            for N_fe=N_FE
                problem_options = nosnoc.Options();
                model = nosnoc.model.Pds();
                model_name = ['PDSOCP'];

                problem_options.rk_scheme = RKSchemes.RADAU_IIA;
                problem_options.rk_representation = RKRepresentation.integral;
                problem_options.cross_comp_mode = cross_comp_mode;
                problem_options.N_finite_elements = N_fe;
                problem_options.n_s = n_s;
                problem_options.N_stages = N_stages;
                problem_options.T = T;
                problem_options.rho_h = 1e-10;
                problem_options.gamma_h = .9999;
                problem_options.use_fesd = true;
                problem_options.gcs_lift_gap_functions = true;


                model = nosnoc.model.Pds();
                x = SX.sym('x',2);
                model.x = [x];
                model.lbx = [-inf;-inf];
                model.ubx = [inf;inf];
                x0 =[1; 5];
                model.x0 = [x0];
                u = SX.sym('u1', 2);;
                model.u = [u];
                model.lbu = [-1;-1];
                model.ubu = [1;1];
                model.u0 = [0;0];
                P = [1/4, 0;
                    0, 1/16];
                model.c = [x'*P*x - 1];
                model.f_x_unconstrained = [-0.2*(x(1)+1)^2;-0.4*(x(2)+3)] + u;

                % costs
                R = diag([1e1;1e1]);
                Q_T = diag([0;0]);
                model.f_q = u'*R*u;
                model.f_q_T = 0.5*(x-x_target)'*Q_T*(x-x_target);
                model.g_terminal = x-x_target;
                %% Generate problem
                filename = generate_problem_name(model_name, model, problem_options, 1);
                %% Save problem
                discrete_time_problem = generate_problem(filename, model, problem_options);
                index = index+1;
            end
        end
    end
end