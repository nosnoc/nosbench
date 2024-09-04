clear all
close all
import casadi.*

LEVEL = 2;

CROSS_COMP_MODES = [1,3,4,7];
N_STAGES = [30,33,50,57];
N_S = [2];
F_FRICTION = [2,5];
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stages=N_STAGES
        for n_s=N_S
            for idx=1:length(F_FRICTION)
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pss();
                    model_name = ['CPWF'];
                    problem_options.rk_scheme = RKSchemes.RADAU_IIA;
                    problem_options.n_s = n_s;
                    problem_options.dcs_mode = dcs_mode;
                    problem_options.N_stages = N_stages; % number of control intervals
                    problem_options.N_finite_elements = 2; % number of finite element on every control intevral
                    problem_options.cross_comp_mode = cross_comp_mode;

                    problem_options.T = 4;    % Time horizon

                    %% Model parameters and defintion
                    q = SX.sym('q', 2);
                    v = SX.sym('v', 2);
                    x = [q;v];
                    u = SX.sym('u', 1); % control

                    F_friction = F_FRICTION(idx);
                    m1 = 1; % cart
                    m2 = 0.1; % link
                    link_length = 1;
                    g = 9.81;
                    % Inertia matrix
                    M = [m1 + m2, m2*link_length*cos(q(2));...
                        m2 *link_length*cos(q(2)),  m2*link_length^2];
                    % Coriolis force
                    C = [0, -m2 * link_length*v(2)*sin(q(2));...
                        0,   0];

                    % f_all (all forces) = Gravity+Control+Coriolis (+Friction)
                    f_all = [0;-m2*g*link_length*sin(x(2))]+[u;0]-C*v;

                    % Dynamics with $ v > 0$
                    f_1 = [v;...
                        inv(M)*(f_all-[F_friction;0])];
                    % Dynamics with $ v < 0$
                    f_2 = [v;...
                        inv(M)*(f_all+[F_friction;0])];
                    F = [f_1, f_2];
                    % switching function (cart velocity)
                    c = v(1);
                    % Sign matrix % f_1 for c=v>0, f_2 for c=v<0
                    S = [1; -1];

                    % specify initial and end state, cost ref and weight matrix
                    x0 = [1; 0/180*pi; 0; 0]; % start downwards
                    x_ref = [0; 180/180*pi; 0; 0]; % end upwards

                    Q = diag([1; 100; 1; 1]);
                    % Q_terminal = diag([10; 100; 10; 20]);
                    Q_terminal = diag([100; 100; 10; 10]);
                    % Q_terminal = 10*Q;
                    R = 1;

                    % bounds
                    ubx = [5; 240/180*pi; 20; 20];
                    lbx = [-0.0; -240/180*pi; -20; -20];
                    u_max = 30;
                    u_ref = 0;

                    %% fill in model
                    model.F = F;
                    model.c = c;
                    model.lbx = lbx;
                    model.ubx = ubx;
                    model.x = x;
                    model.x0 =  x0;
                    model.u = u;
                    model.lbu = -u_max;
                    model.ubu = u_max;
                    model.S = S;

                    % Stage cost
                    if 1
                        % directly via generic stage and terminal costs
                        model.f_q = (x-x_ref)'*Q*(x-x_ref)+ (u-u_ref)'*R*(u-u_ref);
                        % terminal cost
                        model.f_q_T = (x-x_ref)'*Q_terminal*(x-x_ref);
                        % model.g_terminal = x-x_target;
                    else
                        % via least squares cost interace (makes time variable reference possible)
                        model.lsq_x = {x,x_ref,Q};
                        model.lsq_u = {u,u_ref,R};
                        model.lsq_T = {x,x_ref,Q_terminal};
                    end
                    %% Generate problem
                    filename = generate_problem_name(model_name, model, problem_options, idx);
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
