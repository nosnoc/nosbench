clear all
close all
import casadi.*

LEVEL = 4;

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [3];
N_STAGES = [20,23,37,40];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stg=N_STAGES
        for N_fe=N_FE
            problem_options = nosnoc.Options();
            model = nosnoc.model.Cls();
            model_name = ['HOPOCP'];
            %%
            problem_options.rk_scheme = RKSchemes.RADAU_IIA;
            problem_options.n_s = 2;  % number of stages in RK methods
            problem_options.use_fesd = 1;
            problem_options.cross_comp_mode = cross_comp_mode;
            problem_options.time_freezing = 1;
            % problem_options.s_sot_max = 2;
            problem_options.s_sot_min = 1;
            problem_options.equidistant_control_grid = 1;
            problem_options.pss_lift_step_functions = 1;
            problem_options.stagewise_clock_constraint = 1;
            problem_options.g_path_at_fe = 1; % evaluate path constraint on every integration step
            problem_options.g_path_at_stg = 1; % evaluate path constraint on every stage point
            problem_options.a_n = 1e2;
            problem_options.dcs_mode = DcsMode.Heaviside;

            %% discretization
            T = 1; % prediction horizon

            v_slip_bound = 0.001;
            full_comp = 1;
            x_goal = 0.7;
            %% Hopper model
            q = SX.sym('q', 4);
            v = SX.sym('v', 4);
            x = [q;v];
            u = SX.sym('u', 3);
            % state equations
            mb = 1;  % body
            ml = 0.1; % link
            Ib = 0.25; % body
            Il = 0.025; % link
            mu = 0.45;
            g = 9.81;

            % inertia matrix
            M = diag([mb + ml, mb + ml, Ib + Il, ml]);
            % coriolis and graviry
            C = [0;(mb + ml)*g;0;0];
            % Control input matrix
            B = [0, -sin(q(3));...
                0, cos(q(3));...
                1, 0;...
                0, 1];

            % constraint normal
            f_c_normal = [0; 1; q(4)*sin(q(3)); -cos(q(3))];
            % constraint tangent;
            f_c_tangent = [1; 0; q(4)*cos(q(3)); sin(q(3))];

            % tangential and normal velocity of the contact point
            v_tangent = f_c_tangent'*v;
            v_normal = f_c_normal'*v;

            % All forces
            f_v = -C + B*u(1:2);

            % Gap function
            f_c = q(2) - q(4)*cos(q(3));

            % The control u(3) is a slack for modellingo of nonslipping constraints.
            ubu= [50; 50; 100];
            lbu= [-50; -50; 0];

            ubx = [x_goal+0.1; 1.5; pi; 0.50; 10; 10; 5; 5];
            lbx =  [0; 0; -pi; 0.1; -10; -10; -5; -5];

            x0 = [0.1; 0.5; 0; 0.5; 0; 0; 0; 0];
            x_mid = [(x_goal-0.1)/2+0.1; 0.8; 0; 0.1; 0; 0; 0; 0];
            x_end = [x_goal; 0.5; 0; 0.5; 0; 0; 0; 0];

            Q = diag([50; 50; 20; 50; 0.1; 0.1; 0.1; 0.1]);
            Q_terminal =diag([50; 50; 50; 50; 0.1; 0.1; 0.1; 0.1]);

            Q = diag([50; 50; 20; 50; 0.1; 0.1; 0.1; 0.1]);
            Q_terminal =diag([300; 300; 300; 300; 0.1; 0.1; 0.1; 0.1]);

            u_ref = [0; 0; 0];
            R = diag([0.01; 0.01; 1e-5]);

            % Avoid slipping motion
            g_comp_path = [v_tangent,u(3);f_c,u(3)];

            %% interpolate refernece
            x_ref = interp1([0 0.5 1],[x0,x_mid,x_end]',linspace(0,1,N_stg),'spline')'; %spline

            %% Populate model
            problem_options.T = T;
            problem_options.N_stages = N_stg;
            problem_options.N_finite_elements = N_fe;
            model.x = x;
            model.u = u;
            model.e = 0;
            model.mu = mu;
            model.x0 = x0;

            model.M = M;
            model.f_v = f_v;
            % gap functions
            model.f_c = f_c;
            model.J_tangent = f_c_tangent;
            model.dims.n_dim_contact = 1;

            % box constraints on controls and states
            model.lbu = lbu;
            model.ubu = ubu;
            model.lbx = lbx;
            model.ubx = ubx;
            % constraint on drift velocity
            model.G_path = g_comp_path(:,1);
            model.H_path = g_comp_path(:,2);
            % LSQ objective
            model.lsq_x = {x,x_ref,Q};
            model.lsq_u = {u,u_ref,R};
            model.lsq_T = {x,x_end,Q_terminal};
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
