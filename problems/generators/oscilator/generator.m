clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [4];
N_FE = [2];
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];
INITIAL_POINTS = {[exp(-1);0],
    [0.9633;-0.1527]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for idx=1:length(INITIAL_POINTS)
        for n_s=N_S
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pss();
                    model_name = ['OSCIL'];
                    problem_options.T = (pi/2)/29;
                    problem_options.n_s = n_s;                            
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.dcs_mode = dcs_mode;
                    problem_options.N_finite_elements = N_fe;

                    %% Initial value
                    model.x0 = INITIAL_POINTS{idx};
                    model.u0 = 0; % guess for control variables

                    %% settings
                    problem_options.use_fesd = 1;       % switch detection method on/off
                    problem_options.rk_scheme = RKSchemes.RADAU_IIA; %'Gauss-Legendre';
                    
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
                    %% Generate problem
                    filename = generate_problem_name(model_name, model, problem_options, idx);
                    %% Save problem
                    discrete_time_problem = generate_problem(filename, model, problem_options);
                    index = index+1;
                end
            end
        end
    end
end
