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
                    model_name = ['OPNGT'];
                    %% parameter
                    T = 5;
                    R = 1; % radius of manipulators
                    R_obj = R_OBJ(idx); % radius of manipulated object
                    R_obstacle = 5; % radius of obstacle
                    %% Define projected system
                    x1 = SX.sym('x1', 2); % center of manipulator 1
                    x2 = SX.sym('x2', 2); % center of manipulator 2
                    x3 = SX.sym('x3', 2); % center of manipulated object
                    gate = SX.sym('gate', 1); % vertical position of gate, horizontal fix
                    x = [x1;x2;x3;gate];
                    x_target = [-10;3;-10;0;-7;3;0];
                    x0 = [-10;2;10;4;7;3.5;0.9];
                    u1 = SX.sym('u1', 2);
                    u2 = SX.sym('u2', 2);

                    % populate nosnoc PDS model
                    model.x = x;
                    model.lbx = [-inf;-inf;-inf;-inf;-inf;-inf;-inf];
                    model.ubx = [inf;5;inf;5;inf;5;inf];
                    model.x0 = x0;
                    model.u = [u1;u2];
                    model.lbu = [-10;-10;-10;-10];
                    model.ubu = [10;10;10;10];
                    model.u0 = [0;0;0;0];
                    model.c = [norm_2(x3-x1)-(R+R_obj);
                        norm_2(x3-x2)-(R+R_obj);
                        norm_2(x2-x1)-(R+R);
                        x1(2)-gate-R]; % these are the distance functions between manipualtors and objects
                    model.g_path = [x2(2)-gate-R;
                        x3(2)-gate-R_obj;
                        norm_2(x1-[0;5]) - R-R_obstacle;
                        norm_2(x2-[0;5]) - R-R_obstacle;
                        norm_2(x3-[0;5]) - R_obj-R_obstacle]; % path constraints for obstacle avoidance and not touching the gate
                    model.lbg_path = [0;0;0;0;0];
                    model.ubg_path = [inf;inf;inf;inf;inf];
                    model.f_x_unconstrained = [u1;u2;0;0;0]; % dynamics of the PDS when no constraints are active

                    % costs
                    model.f_q = 1e-4*norm_2(model.u)^2;
                    model.f_q_T = (x-x_target)'*diag([1,1,1,1,1e3,1e3,0])*(x-x_target);
                    %model.g_T = x3 - [-7;0];

                    % Time discertization settings

                    problem_options.T = T;
                    problem_options.N_stages = N_stages;
                    problem_options.N_finite_elements = N_fe;
                    problem_options.n_s = n_s;
                    problem_options.cross_comp_mode = cross_comp_mode;


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