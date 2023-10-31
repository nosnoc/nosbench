clear all
close all
import casadi.*

LEVEL = 3;

CROSS_COMP_MODES = [1,3,4,7];
N_STAGES = [23,30,37];
N_S = [2];
N_FE = [3];
NONLINEAR = [0,1];

% Drone Landing Problem
% example from "iLQR for Piecewise-Smooth Hybrid Dynamical Systems" by Nathan J. Kong, George Council, and Aaron M. Johnson
% 2021, 60th IEEE Conference on Decision and Control (CDC)

index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for N_stages=N_STAGES
            for n_s=N_S
                for N_fe=N_FE
                    for nonlinear_constraint=NONLINEAR
                        problem_options = NosnocProblemOptions();
                        model = NosnocModel();
                        model.model_name = ['DRNLND'];
                        
                        %% Discretization
                        problem_options.T = 2;
                        problem_options.N_finite_elements = N_fe;
                        problem_options.N_stages = N_stages;
                        problem_options.n_s = n_s;

                        problem_options.cross_comp_mode = cross_comp_mode;
                        problem_options.time_optimal_problem = 0;
                        problem_options.use_fesd = 1;
                        problem_options.pss_lift_step_functions = 1;
                        problem_options.time_freezing = 1;
                        problem_options.stagewise_clock_constraint = 1;
                        problem_options.s_sot_max = 2;
                        problem_options.s_sot_min = 1;
                        problem_options.lift_complementarities = lift;

                        %% Planar drone model
                        q = SX.sym('q',3);
                        v = SX.sym('v',3);
                        u = SX.sym('u',2);
                        x = [q;v];
                        u_L = u(1);
                        u_R = u(2);
                        y = q(1);
                        z = q(2);
                        theta = q(3);

                        model.x = x;
                        model.u = u;
                        model.e = 0;
                        model.mu = 0.0;
                        model.a_n = 25;

                        m = 1;
                        I = 1;
                        g = 9.81;
                        M = [m 0 0;0 m 0; 0 0 I];
                        w = 0.5; % width of the drone
                                 % kinematics
                        y_L = y-0.5*w*cos(theta);
                        z_L = z-0.5*w*sin(theta);
                        y_R = y+0.5*w*cos(theta);
                        z_R = z+0.5*w*sin(theta);

                        % normal vectors
                        y_Ln = 1;
                        % z_Ln = -y_L/z_L;
                        z_Ln = -(y_L-y)/(z_L-z);
                        norm_N_L = norm([y_Ln;z_Ln]);
                        N_L = [y_Ln/norm_N_L;z_Ln/norm_N_L];
                        N_L_fun = Function('N_L_fun',{y,z,theta},{N_L});

                        y_Rn = 1;
                        % z_Rn = -y_R/z_R;
                        z_Rn = -(y_R-y)/(z_R-z);
                        norm_N_R = norm([y_Rn;z_Rn]);
                        N_R = [y_Rn/norm_N_R;z_Rn/norm_N_R];
                        N_R_fun = Function('N_R_fun',{y,z,theta},{N_R});

                        % inital value and dynamics
                        x0 = [2;2.5;-pi/8*1;...
                            4;0;0];

                        if nonlinear_constraint
                            x0 = [3;2.0;-pi/8*1;...
                                4;0;0];
                        else
                            x0 = [-1;3.0;pi/8;...
                                0;0;0];
                        end
                        % radius
                        R = 5;
                        % gravity
                        G = [0;-m*g;0];
                        % control terms
                        B = [-sin(theta)*(u_L+u_R);cos(theta)*(u_L+u_R);0.5*w*(u_R-u_L)];
                        % all forces
                        f = G+B;

                        % Gap functions and targent
                        if nonlinear_constraint
                            f_c = [R^2-z_R^2-y_R^2;...
                                R^2-z_L^2-y_L^2];
                            x_target = [R*cos(-pi/12);R*sin(-pi/12);-7*pi/12;...
                                0;0;0];
                            idx = 2;
                        else
                            f_c = [z_L;z_R];
                            x_target = [2;0;0;...
                                0;0;0];
                            idx = 1;
                        end

                        % Refernece and cost
                        x_ref = interp1([0 1],[x0,x_target]',linspace(0,1,N_stages),'spline')'; %spline
                        u_ref = [0;0];
                        R_control = diag([0.1 0.1]);
                        Q =  diag([10; 10; 10; 0.1; 0.1; 0.1]);
                        Q_terminal = Q*50;

                        %% model struct
                        model.x0 = x0;
                        model.f_v = f;
                        model.M = M;
                        model.f_c = f_c;

                        %% Discretization and ocp
                        u_max = 30*ones(2,1);

                        model.lbu = -u_max;
                        model.ubu = u_max;
                        % model.g_terminal = [x-x_target];

                        % reference and least square cost
                        model.lsq_x = {x,x_ref,Q};
                        model.lsq_u = {u,u_ref,R_control};
                        model.lsq_T = {x,x_target,Q_terminal};


                        %% Solve OCP
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
end
