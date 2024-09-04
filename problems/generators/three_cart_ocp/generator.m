clear all
close all
import casadi.*

LEVEL = 3;

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [3];
N_STAGES = [20, 30];
CART_MASS = [1.0, 0.5, 2.0];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stg=N_STAGES
        for N_fe=N_FE
            for idx=1:length(CART_MASS)
                problem_options = nosnoc.Options();
                model = nosnoc.model.Cls();
                model_name = ['3CPTF'];

                %%
                problem_options.rk_scheme = RKSchemes.RADAU_IIA;
                problem_options.n_s = 2;  % number of stages in RK methods
                problem_options.time_freezing = 1;

                problem_options.cross_comp_mode = cross_comp_mode;
                problem_options.a_n = 20;
                problem_options.dcs_mode = DcsMode.Heaviside;

                %% model parameters
                m_cart = CART_MASS(idx);
                m1 = m_cart;
                m2 = 1;
                m3 = m_cart;
                cart_width1 = 2;
                cart_width2 = 2;
                cart_width3 = 2;
                k1 = 1*0;
                k2 = 1*0;
                k3 = 1*0;
                c_damping = 2;

                ubx = [10; 15; 15; 5; 5; 5]; 
                lbx = [-15; -15; -10; -5; -5; -5];            
                
                x0 = [-3; 0; 3; 0; 0; 0];
                x_ref = [-5; 0; 5; 0; 0; 0];
                u_ref = [0; 0; 0];

                Q = diag([10; 1; 10; 0.1; 0.1; 0.1]);
                Q_terminal = 100*Q;
                R = diag([0.1; 0.1; 0.1]);

                u_max = 20;
                u_min = -20;

                lbu = [0*u_min; u_min; 0*u_min];
                ubu = [0*u_max; u_max; 0*u_max];

                %% Symbolic variables and bounds
                q = SX.sym('q',3); v = SX.sym('v',3); 
                u = SX.sym('u',3);
                x = [q;v];
                problem_options.T = 4;
                problem_options.N_stages = N_stg;
                problem_options.N_finite_elements = N_fe;
                model.x = x;
                model.u = u;
                model.e = 0;
                model.mu = 0.0;
                model.x0 = x0; 

                model.M = diag([m1;m2;m3]); % inertia/mass matrix;
                model.f_v = [(u(1)-c_damping*v(1)-k1*q(1));...
                    (u(2)-c_damping*v(2)+k2*(q(1)-q(2)));...
                    (u(3)-c_damping*v(3)+k3*q(2))];

                % gap functions
                model.f_c = [q(2) - q(1) - 0.5*cart_width2 - 0.5*cart_width1;...
                    q(3) - q(2) - 0.5*cart_width3 - 0.5*cart_width2];
                model.dims.n_dim_contact = 2;

                % box constraints on controls and states
                model.lbu = lbu;
                model.ubu = ubu;
                model.lbx = lbx;
                model.ubx = ubx;
                % Stage cost
                model.f_q = (x-x_ref)'*Q*(x-x_ref)+ u'*R*u;
                % terminal cost
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
                fprintf(fid, '%s', json);
                fclose(fid);
                index = index+1;
            end
        end
    end 
end
