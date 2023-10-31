clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [4];
N_FE = [2];
DCS_MODES = [DcsMode.Step, DcsMode.Stewart];
INITIAL_POINTS = {[exp(-1);0],
    [0.9633;-0.1527]};

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
                        model.model_name = ['OSCIL'];
                        problem_options.T = (pi/2)/29;
                        problem_options.n_s = n_s;                            
                        problem_options.cross_comp_mode = cross_comp_mode;
                        problem_options.dcs_mode = dcs_mode;
                        problem_options.N_finite_elements = N_fe;
                        problem_options.lift_complementarities = lift;
                        %% Initial value
                        model.x0 = INITIAL_POINTS{idx};
                        model.u0 = 0; % guess for control variables

                        %% settings
                        problem_options.use_fesd = 1;       % switch detection method on/off
                        problem_options.irk_scheme = IRKSchemes.RADAU_IIA; %'Gauss-Legendre';
                        
                        %% Time settings
                        omega = -2*pi;
                        A1 = [1 omega;...
                            -omega 1];
                        A2 = [1 -omega;...
                            omega 1];
                        % Variable defintion
                        x1 = SX.sym('x1');
                        x2 = SX.sym('x2');
                        x = [x1;x2];
                        c = x1^2+x2^2-1;
                        model.x = x;
                        model.c = c;
                        model.S = [-1;1];
                        f_11 = A1*x;
                        f_12 = A2*x;
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
