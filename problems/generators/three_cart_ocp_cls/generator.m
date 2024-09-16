clear all
close all
import casadi.*

LEVEL = 4;

CROSS_COMP_MODES = [1,3,4,7];
N_FE = [3];
N_STAGES = [15,30];
CART_MASS = [1.0, 0.5, 2.0];

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stg=N_STAGES
        for N_fe=N_FE
            for idx=1:length(CART_MASS)
                problem_options = nosnoc.Options();
                model = nosnoc.model.Cls();
                model_name = ['3CPCLS'];

                %%
                problem_options.rk_scheme = RKSchemes.RADAU_IIA;
                problem_options.n_s = 2;  % number of stages in RK methods
                problem_options.dcs_mode = 'CLS';
                problem_options.cross_comp_mode = cross_comp_mode;
                problem_options.friction_model = "Polyhedral";


                problem_options.gamma_h = 0.8;

                % NLP solver settings;
                problem_options.eps_cls = 0;

                problem_options.N_stages = N_stg;
                problem_options.N_finite_elements  = N_fe;
                problem_options.T = 6;

                %% model parameters
                m_cart = CART_MASS(idx);
                m1 = m_cart;
                m2 = 1;
                m3 = m_cart;
                cart_width1 = 2;
                cart_width2 = 2;
                cart_width3 = 2;

                M = diag([m1, m1, m2, m2, m3, m3]);

                ubx = ones(12,1)*10;
                lbx = -ones(12,1)*10;
                ubu = 30;
                lbu = -30;

                x0 = [ -3; 1; 0; 1;  3; 1; ...
                    0; 0; 0; 0; 0; 0];
                u_ref = 0;

                x_ref = [-7; 1; 0; 1; 5; 1;...
                    0; 0; 0; 0; 0; 0];
                
                Q = diag([10; 0.001; 1; 0.001; 10; 0.001; ...
                           0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);
                Q_terminal = 100*Q;
                R = 0.1;

                %% Symbolic variables and bounds
                g = 9.81;
                q = SX.sym('q',6);
                v = SX.sym('v',6);
                u = SX.sym('u',1);
                x = [q;v];

                q1 = q(1:2);
                q2 = q(3:4);
                q3 = q(5:6);

                model.x = x;
                model.u = u;
                model.x0 = x0;

                model.M = M;
                model.f_v = [ 0;...
                    -m1*g;
                    u;...
                    -m2*g;
                    0;...
                    -m3*g];

                % gap functions
                f_c = [q2(1) - q1(1) - 0.5*cart_width2 - 0.5*cart_width1;...
                    q3(1) - q2(1) - 0.5*cart_width3 - 0.5*cart_width2;...
                    q1(2)-cart_width1/2;...
                    q2(2)-cart_width2/2;...
                    q3(2)-cart_width3/2;...
                      ];

                J_tangent = [0  0 1 0 0 ;...
                    -1 -1 0 0 0;...
                    0  0 0 1 0;...
                    1  0 0 0 0;...
                    0  0 0 0 1;...
                    0  1 0 0 0];

                J_tangent =   J_tangent./vecnorm(J_tangent);

                J_normal = full(f_c.jacobian(q));
                J_normal_fun = Function('J_normal_fun',{q},{J_normal});
                J_normal = full(J_normal_fun(x0(1:6)))';
                J_normal  = J_normal ./vecnorm(J_normal);

                D_tangent  = [];
                for ii = 1:size(J_tangent,2)
                    D_tangent  = [D_tangent, J_tangent(:,ii), -J_tangent(:,ii)];
                end

                model.f_c = f_c;
                model.J_normal = J_normal;
                model.J_tangent = J_tangent;
                model.D_tangent = D_tangent;
                model.e =  [0.0 0.5 0.0 0.0 0.0];
                model.mu = [0.1 0.1 0.2 0.2 0.2];
                % box constraints on controls and states
                model.lbu = lbu;
                model.ubu = ubu;
                model.lbx = lbx;
                model.ubx = ubx;
                % Stage cost
                model.f_q = (x-x_ref)'*Q*(x-x_ref)+ u'*R*u;
                model.f_q_T = (x-x_ref)'*Q_terminal*(x-x_ref);

                %% Generate problem
                filename = generate_problem_name(model_name, model, problem_options, idx);
                %% Save problem
                discrete_time_problem = generate_problem(filename, model, problem_options);
                index = index+1;
            end
        end
    end
end
