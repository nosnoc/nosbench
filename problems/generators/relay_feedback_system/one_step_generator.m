clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [2];
N_FE = [2];
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];
INITIAL_POINTS = {[-0.000000002424356;-0.938831521258458;-2.586020941994411],
    [0.000302680050006;0.982013428031944;-1.731565277560998],
    [0;-0.001;-0.02]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for idx=1:length(INITIAL_POINTS)
        for n_s=N_S
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pss();
                    model_name = ['RFB1S'];
                    problem_options.use_fesd = 1;
                    problem_options.rk_scheme = RKSchemes.RADAU_IIA; %IRKSchemes.GAUSS_LEGENDRE;
                    problem_options.n_s = 2;
                    problem_options.dcs_mode = dcs_mode; % 'Step;
                    problem_options.cross_comp_mode = cross_comp_mode;

                    %% Time settings
                    problem_options.T = 0.05;
                    problem_options.N_finite_elements = N_fe;
                    %% Model
                    model.x0 = INITIAL_POINTS{idx};
                    % Variable defintion
                    x = SX.sym('x',3);
                    omega = 25;
                    xi = 0.05;
                    sigma = 1;

                    A = [-(2*xi*omega+1)        1 0;...
                        -(2*xi*omega+omega^2)  0 1;...
                        -omega^2               0 0];
                    B = [1; -2*sigma;1];
                    c = x(1);
                    f_11 = A*x+B;
                    f_12 = A*x-B;

                    model.x = x;
                    model.c = c;
                    model.S = [-1;1];

                    F = [f_11 f_12];
                    model.F = F;
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
                    fprintf(fid, '%s', casadi_json);
                    fclose(fid);
                    index = index+1;
                end
            end
        end
    end
end
