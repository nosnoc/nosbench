clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [2];
N_FE = [2];
DCS_MODES = [DcsMode.Step, DcsMode.Stewart];
INITIAL_POINTS = {[-0.000000002424356;-0.938831521258458;-2.586020941994411],
    [0.000302680050006;0.982013428031944;-1.731565277560998],
    [0;-0.001;-0.02]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for idx=1:length(INITIAL_POINTS)
            for n_s=N_S
                for N_fe=N_FE
                    for dcs_mode=DCS_MODES
                        problem_options = NosnocProblemOptions();
                        model = NosnocModel();
                        model.model_name = ['RFB1S'];
                        problem_options.use_fesd = 1;
                        problem_options.irk_scheme = IRKSchemes.RADAU_IIA; %IRKSchemes.GAUSS_LEGENDRE;
                        problem_options.n_s = 2;
                        problem_options.dcs_mode = dcs_mode; % 'Step;
                        problem_options.cross_comp_mode = cross_comp_mode;
                        problem_options.lift_complementarities = lift;
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
                        %% Solve OCP
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
end
