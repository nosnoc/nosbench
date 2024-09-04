clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [3];
N_FE = [2];
CONDS = {[1;2;0;0]
    [0.2152;1.2152;-3.9240;-3.9240],
    [0.2119;1.1741;-2.8636;1.8424]};
% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for n_s=N_S
        for N_fe=N_FE
            for idx=1:length(CONDS)
                problem_options = nosnoc.Options();
                model = nosnoc.model.Cls();
                model_name = ['2BCLS'];
                

                %% Model Settings
                problem_options.N_finite_elements = N_fe;
                problem_options.n_s = n_s;
                problem_options.T = 0.005;

                %%
                problem_options.rk_scheme = RKSchemes.GAUSS_LEGENDRE;
                problem_options.n_s = 3;
                problem_options.cross_comp_mode = cross_comp_mode;
                problem_options.dcs_mode = DcsMode.CLS;
                problem_options.no_initial_impacts = 1;

                %% Generate Model
                g = 9.81;
                R = 0.2;
                k = 1e4;
                l = 1;
                m = 1;
                % Symbolic variables and bounds
                q = SX.sym('q',2);
                v = SX.sym('v',2);
                model.M = eye(2);
                model.x = [q;v];
                model.e = 0.8;
                model.mu = 0;
                model.x0 = CONDS{idx};
                model.f_v = [-m*g+k*(q(2)-q(1)-l);-m*g-k*(q(2)-q(1)-l)];
                model.f_c = q(1)-R;

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
