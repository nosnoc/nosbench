clear all
close all
import casadi.*

N_fe = 32;
T = 1.5;

LEVEL = 1;
for dcs_mode = [DcsMode.Stewart, DcsMode.Step]
    for cross_comp_mode=[1,3,4,7]
        %% crossing
        problem_options = nosnoc.Options();
        model = NosnocModel();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.irk_scheme = IRKSchemes.GAUSS_LEGENDRE;
        problem_options.irk_representation= 'differential';
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
        model.model_name = 'SMCRS';

        problem_options.preprocess();
        model.verify_and_backfill(problem_options);
        model.generate_variables(problem_options);
        model.generate_equations(problem_options);
        filename = generate_problem_name(model, problem_options, 1)
        save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');

        %% sliding_mode
        problem_options = nosnoc.Options();
        model = NosnocModel();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.cross_comp_mode = cross_comp_mode;
        problem_options.irk_scheme = IRKSchemes.GAUSS_LEGENDRE;
        problem_options.irk_representation= 'differential';
        problem_options.T = T;
        problem_options.dcs_mode = dcs_mode;

        model.x0 = -0.5;
        x = SX.sym('x',1);
        model.x = x;
        model.c = x;
        model.S = [-1; 1];
        f_1 = [1]; f_2 = [-1];
        model.F = [f_1 f_2];
        model.model_name = 'SMSLM';

        problem_options.preprocess();
        model.verify_and_backfill(problem_options);
        model.generate_variables(problem_options);
        model.generate_equations(problem_options);
        filename = generate_problem_name(model, problem_options, 1)
        save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');

        %% spontaneous switch
        problem_options = nosnoc.Options();
        model = NosnocModel();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.cross_comp_mode = cross_comp_mode;
        problem_options.irk_scheme = IRKSchemes.EXPLICIT_RK;
        problem_options.irk_representation= 'differential';
        problem_options.T = T;
        problem_options.dcs_mode = dcs_mode;

        model.x0 = 0;
        x = SX.sym('x',1);
        model.x = x;
        model.c = x;
        model.S = [-1; 1];
        f_1 = [-1]; f_2 = [1];
        model.F = [f_1 f_2];
        model.model_name = 'SMSPS';

        problem_options.preprocess();
        model.verify_and_backfill(problem_options);
        model.generate_variables(problem_options);
        model.generate_equations(problem_options);
        filename = generate_problem_name(model, problem_options, 1)
        save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');

        %% leave_sliding_mode
        problem_options = nosnoc.Options();
        model = NosnocModel();
        problem_options.n_s = 2;
        problem_options.N_finite_elements = N_fe;
        problem_options.cross_comp_mode = cross_comp_mode;
        problem_options.irk_scheme = IRKSchemes.GAUSS_LEGENDRE;
        problem_options.irk_representation= 'differential';
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
        model.model_name = 'SMLSM';

        problem_options.preprocess();
        model.verify_and_backfill(problem_options);
        model.generate_variables(problem_options);
        model.generate_equations(problem_options);
        filename = generate_problem_name(model, problem_options, 1)
        save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');
    end
end