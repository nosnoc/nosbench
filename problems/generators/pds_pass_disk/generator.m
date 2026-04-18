clear all
close all
import casadi.*

LEVEL = 3;

CROSS_COMP_MODES = [7];
N_STAGES = [20, 40];
N_S = [2];
N_FE = [3];
R_OBJ = [0.5,1.0,1.5];


% An optimal control problem from:
% Towards Solutions of Manipulation Tasks via Optimal Control of 
% Projected Dynamical Systems
% Anton Pozharskiy, Armin Nurkanovic, Moritz Diehl

index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stages=N_STAGES
        for n_s=N_S
            for N_fe=N_FE
                for idx=1:length(R_OBJ)
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pds();
                    model_name = ['PSDSK'];

                    %% Parameters
                    T = 5;  % time horizon
                    R = 1; % radius of manipulators
                    R_obj = R_OBJ(idx); % radius of manipulated object
                    %% Define projected system
                    x1 = SX.sym('x1', 2); % center of manipulator disc 1
                    x2 = SX.sym('x2', 2); % center of manipulator disc 2
                    x3 = SX.sym('x3', 2); % center of manipulated disc
                    x = [x1;x2;x3];
                    x0 = [-10;10;5;-3;0;0];
                    x_target = [-10;0;10;0;0;10];
                    % control "forces" for manipulators
                    u1 = SX.sym('u1', 2);
                    u2 = SX.sym('u2', 2);

                    % Populate nosnoc PDS model
                    model.x = x;
                    model.lbx = [-inf;-inf;R_obj+0.5;-inf;-inf;-inf];
                    model.ubx = [-(R_obj+0.5);inf;inf;inf;inf;inf];
                    model.x0 = x0;
                    model.u = [u1;u2];
                    model.lbu = [-10;-10;-10;-10];
                    model.ubu = [10;10;10;10];
                    model.u0 = [0;0;0;0];
                    model.c = [norm_2(x3-x1)-(R+R_obj);norm_2(x3-x2)-(R+R_obj)];
                    model.f_x_unconstrained = [u1;u2;0;0];

                    % costs
                    model.f_q = 1e-4*norm_2(model.u)^2 + (x-x_target)'*diag([0.0,0.0,0.0,0.0,1e-2,1e-2])*(x-x_target);
                    model.f_q_T = (x-x_target)'*diag([1e1,1e1,1e1,1e1,1e3,1e3])*(x-x_target);

                    % Time discertization settings
                    problem_options.T = T;
                    problem_options.N_stages = N_stages; % numbe of control stages\intervals
                    problem_options.N_finite_elements = N_fe; % number of integration steps in every control interval
                    problem_options.n_s = n_s;
                    problem_options.cross_comp_mode = "FE_FE";



                    %% Generate problem
                    filename = generate_problem_name(model_name, model, problem_options, idx);
                    %% Save problem
                    discrete_time_problem = generate_problem(filename, model, problem_options);
                    index = index+1;
                end
            end
        end
    end
end