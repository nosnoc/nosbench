clear all
close all
import casadi.*

LEVEL = 3;

CROSS_COMP_MODES = [1,3,4,7];
N_STAGES = [50, 80];
N_S = [4];
N_FE = [2];
PATH = ["linear", "nonlinear", "track"];
TIME_OPTIMAL = [0,1];
CONDS = {"chicane", 0;"nonlinear", 0;"track",0;"chicane",1;"nonlinear",1;"track",1};
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];


% An optimal control problem from: 
% Optimal control of systems with discontinuous differential equations 
% April 2012, Numerische Mathematik 114(4):653-695
% DOI: 10.1007/s00211-009-0262-2
% David E. Stewart Mihai Anitescu

index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for N_stages=N_STAGES
        for n_s=N_S
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    for idx=1:length(CONDS)
                        problem_options = nosnoc.Options();
                        model = nosnoc.model.Pss();
                        model_name = ['SCHUMI'];

                        
                        %% Discretization
                        problem_options.N_finite_elements = N_fe;
                        problem_options.N_stages = N_stages;
                        problem_options.n_s = n_s;
                        problem_options.cross_comp_mode = cross_comp_mode;
                        problem_options.dcs_mode = dcs_mode;

                        problem_options.time_optimal_problem = CONDS{idx,2};
                        problem_options.T_final_max = 5*pi;
                        problem_options.T_final_min = 2;

                        
                        %% model equations
                        track_width = 0.5;
                        model.x0 = zeros(5,1);
                        
                        %% Variable defintion
                        % Declare model variables
                        qx = SX.sym('qx'); qy = SX.sym('qy');
                        vx = SX.sym('vx'); vy = SX.sym('vy');
                        alpha = SX.sym('alpha');
                        tangent = [cos(alpha);sin(alpha)];
                        normal = [-sin(alpha);cos(alpha)];
                        q = [qx;qy];
                        v = [vx;vy];
                        x = [q;v;alpha];
                        model.x = x;

                        %%  control
                        a = SX.sym('a');
                        s = SX.sym('s');
                        u = [a;s];
                        u_max  = 2;
                        model.u = u;
                        model.lbu = -u_max*ones(2,1);
                        model.ubu = u_max*ones(2,1);
                        model.u0 = [u_max;0];
                        %% modes of the PSS
                        Friction_max = 4;
                        f_1 = [v;a*tangent+Friction_max*normal;s*(tangent'*v)];
                        f_2 = [v;a*tangent-Friction_max*normal;s*(tangent'*v)];
                        % Switching functions and modes
                        model.c = normal'*v;
                        model.S = [1;-1];
                        model.F = [f_1 f_2];
                        %%  general nonlinear constinrst
                        % model.g_path = qy-sin(qx);
                        path_constraint = CONDS{idx,1};
                        switch path_constraint 
                          case 'linear'
                            model.g_path = qy-(qx);
                            q_target = [3*pi;3*pi];
                            xx = linspace(0,q_target(1),problem_options.N_stages);
                            yy = xx;
                          case 'nonlinear'
                            omega = 3/8;
                            omega = 1; 
                            %         omega = 0.3;
                            model.g_path = qy-sin(omega*qx);
                            q_target = [3*pi;sin(omega*3*pi)];
                            xx = linspace(0,q_target(1),problem_options.N_stages);
                            yy = sin(omega*xx);
                          case 'track'
                            arg1 = qx-pi;
                            arg2 = qx-2*pi;
                            sig = 1e-1;
                            step1 = 0.5*(1+tanh(arg1/sig));
                            step2 = 0.5*(1+tanh(arg2/sig));
                            model.g_path = qy-(sin(qx)*(1-step1)+(pi-qx)*step1*(1-step2)+(-pi-sin(qx))*step2);
                            q_target = [3*pi;-pi];
                            xx = linspace(0,q_target(1),problem_options.N_stages);
                            yy = sin(xx).*(1-0.5.*(1+tanh((xx-pi)/sig))) +...
                                 (pi-xx).*0.5.*(1+tanh((xx-pi)/sig)).*(1-0.5.*(1+tanh((xx-2*pi)/sig)))+...
                                 (-pi-sin(xx)).*0.5.*(1+tanh((xx-2.*pi)/sig));
                          case 'chicane'
                            chicane_tightness = 1;
                            chicane_width = 2;
                            q_target = [10;2*chicane_width];
                            model.g_path = qy-((chicane_width)+chicane_width*tanh(chicane_tightness*(qx-q_target(1)/2)));
                            xx = linspace(0,q_target(1),problem_options.N_stages);
                            yy = (chicane_width)+chicane_width*tanh(chicane_tightness*(xx-q_target(1)/2));
                        end
                        if ~isequal(path_constraint,'none')
                            model.lbg_path = [-track_width];
                            model.ubg_path = [+track_width];
                        end
                        %% objective
                        if ~problem_options.time_optimal_problem 
                            model.f_q = u'*u;
                            problem_options.T = 5;
                        else
                            model.f_q = 0.01*u'*u;
                            problem_options.T = 5;
                        end

                        % Terminal Constraint
                        model.g_terminal = [q-q_target];

                        % Constrainits at stage points
                        problem_options.g_path_at_fe = 1;
                        problem_options.g_path_at_stg = 1;
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
end
