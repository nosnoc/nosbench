clear all
close all
import casadi.*

N_fe = 32;
T = 1.5;

LEVEL = 1;
for dcs_mode = [DcsMode.Stewart, DcsMode.Heaviside]
    for cross_comp_mode=[1,3,4,7]
        %% crossing
        problem_options = nosnoc.Options();
        model = nosnoc.model.Pss();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.rk_scheme = RKSchemes.GAUSS_LEGENDRE;
        problem_options.rk_representation= 'differential';
        problem_options.cross_comp_mode = cross_comp_mode;
        problem_options.dcs_mode = dcs_mode;
        problem_options.T = T;
        model.x0 = -1;
        x = SX.sym('x',1);
        model.x = x;
        model.c = x;
        model.S = [-1; 1];
        f_1 = [2]; f_2 = [0.2];
        model.F = [f_1 f_2];
        model_name = 'SMCRS';

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
        
        %% sliding_mode
        problem_options = nosnoc.Options();
        model = nosnoc.model.Pss();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.cross_comp_mode = cross_comp_mode;
        problem_options.rk_scheme = RKSchemes.GAUSS_LEGENDRE;
        problem_options.rk_representation= 'differential';
        problem_options.T = T;
        problem_options.dcs_mode = dcs_mode;

        model.x0 = -0.5;
        x = SX.sym('x',1);
        model.x = x;
        model.c = x;
        model.S = [-1; 1];
        f_1 = [1]; f_2 = [-1];
        model.F = [f_1 f_2];
        model_name = 'SMSLM';
        
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
        
        %% spontaneous switch
        problem_options = nosnoc.Options();
        model = nosnoc.model.Pss();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.cross_comp_mode = cross_comp_mode;
        problem_options.rk_scheme = RKSchemes.EXPLICIT_RK;
        problem_options.rk_representation= 'differential';
        problem_options.T = T;
        problem_options.dcs_mode = dcs_mode;

        model.x0 = 0;
        x = SX.sym('x',1);
        model.x = x;
        model.c = x;
        model.S = [-1; 1];
        f_1 = [-1]; f_2 = [1];
        model.F = [f_1 f_2];
        model_name = 'SMSPS';

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

        %% leave_sliding_mode
        problem_options = nosnoc.Options();
        model = nosnoc.model.Pss();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.cross_comp_mode = cross_comp_mode;
        problem_options.rk_scheme = RKSchemes.GAUSS_LEGENDRE;
        problem_options.rk_representation= 'differential';
        problem_options.T = T;
        problem_options.dcs_mode = dcs_mode;
        model.x0 = [0;0];
        x = SX.sym('x',1);
        t = SX.sym('t',1);
        model.x = [x;t];
        model.c = x;
        model.S = [-1; 1];
        f_1 = [1+t;1]; f_2 = [-1+t;1];
        model.F = [f_1 f_2];
        model_name = 'SMLSM';

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
    end
end
