clear all
close all
import casadi.*

LEVEL = 2;

CROSS_COMP_MODES = [1,3,4,7];
N_STAGES = [27,30,50,53];
N_S = [2];
N_FE = [3];
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stages=N_STAGES
        for n_s=N_S
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pss();
                    model_name = ['MWFOCP'];
                    % Choosing the Runge - Kutta Method and number of stages
                    problem_options.rk_scheme = RKSchemes.RADAU_IIA;
                    problem_options.n_s = n_s;
                    % Discretization parameters
                    problem_options.N_stages = N_stages; % number of control intervals
                    problem_options.N_finite_elements = N_fe; % number of finite element on every control intevral (optionally a vector might be passed)
                    problem_options.dcs_mode = dcs_mode;
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.T = 0.08;    % Time horizon


                    %% The Model
                    % Parameters
                    m1 = 1.03; % slide mass
                    m2 = 0.56; % load mass
                    k = 2.4e3; %  spring constant N/m
                    c = 0.00; % damping
                    U_max = 5; % voltage Back-EMF, U = K_s*v_1;
                    R  = 2; % coil resistance ohm
                    L = 2e-3; % inductivity, henry
                    K_F = 12; % force constant N/A ; F_L = K_F*I; % Lorenz force
                    K_S = 12; % Vs/m (not provided in the paper above)
                    F_R = 2.1; % guide friction force, N
                    x0 = [0;0;0;0;0];

                    %% Symbolic variables
                    x1 = SX.sym('x1');  % motor mass position
                    v1 = SX.sym('v1'); % motor mass velocity
                    x2 = SX.sym('x2');  % load mass position
                    v2 = SX.sym('v2'); % load mass velocity
                    I = SX.sym('I'); % electric current
                    x = [x1;v1;x2;v2;I];
                    model.x = x;
                    model.x0 = x0;
                    % control
                    U = SX.sym('U'); % the motor voltage
                    u = [U];
                    n_u = 1;
                    model.u = u;
                    model.lbu = -U_max*ones(n_u,1);
                    model.ubu = U_max*ones(n_u,1);


                    %% Dynamics

                    A = [0  1   0   0   0;...
                        -k/m1 -c/m1 k/m1 c/m1 K_F/m1;...
                        0   0   0   1   0;...
                        k/m2   c/m2   -k/m2   -c/m2 0;...
                        0 -K_S/L    0       0   -R/L];
                    B = [zeros(4,1);1/L];
                    C1 = [0;-F_R/m1;0;0;0]; % v1 >0
                    C2 = -C1; %v1<0

                    % switching dynamics with different friction froces
                    f_1 = A*x+B*u+C1; % v1>0
                    f_2 = A*x+B*u+C2; % v1<0

                    % All modes
                    F = [f_1, f_2];
                    % Switching function
                    c1 = v1;
                    % Sign matrix (pass a cell when having indepdented subsystems)
                    model.S = [1;-1];
                    % The various modes
                    model.F = F;
                    % The switching functions
                    model.c = c1;
                    % OCP
                    x_target = [0.01;0;0.01;0;0];
                    % Stage cost
                    model.f_q = u^2;
                    model.g_terminal = x-x_target;
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
                    fprintf(fid, '%s', json);
                    fclose(fid);
                    index = index+1;
                end
            end
        end
    end
end
