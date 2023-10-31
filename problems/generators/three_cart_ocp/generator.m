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
    for lift=[false, true]
        for N_stg=N_STAGES
            for N_fe=N_FE
                for idx=1:length(CART_MASS)
                    problem_options = NosnocProblemOptions();
                    model = NosnocModel();
                    model.model_name = ['3CPTF'];

                    %%
                    problem_options.irk_scheme = IRKSchemes.RADAU_IIA;
                    problem_options.n_s = 1;  % number of stages in IRK methods
                    problem_options.time_freezing = 1;
                    problem_options.lift_complementarities = lift;
                    problem_options.cross_comp_mode = cross_comp_mode;

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
                    model.mu_f = 0.0;
                    model.a_n = 20;
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
                    
                    %% generate mpcc
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
