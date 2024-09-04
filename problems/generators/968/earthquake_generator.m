clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [2];
N_FE = [3];
DCS_MODES = [DcsMode.Step, DcsMode.Stewart];
INITIAL_POINTS = {[-1e-12;-1e-12;0],
    [0.0045;0.1673;0.4020]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false,true]
        for idx=1:length(INITIAL_POINTS)
            for n_s=N_S
                for N_fe=N_FE
                    for dcs_mode=DCS_MODES
                        problem_options = nosnoc.Options();
                        model = NosnocModel();
                        model.model_name = ['986EQ'];
                        problem_options.n_s = n_s;
                        problem_options.cross_comp_mode = cross_comp_mode;
                        problem_options.dcs_mode = dcs_mode;
                        problem_options.N_finite_elements = N_fe;
                        problem_options.lift_complementarities = lift;
                        model.x0 = INITIAL_POINTS{idx};
                        
                        problem_options.use_fesd = 1;       % switch detection method on/off
                        problem_options.irk_scheme = 'GAUSS_LEGENDRE';

                        %% Time settings
                        k=210.125;
                        c_erthq=2.47e6;
                        % c_erthq=2.47e1;
                        nu=0.005;
                        % Inital Value
                        % x0 = [+0.21;+0.5;0];
                        % Variable defintion
                        y1 = SX.sym('y1');
                        y2 = SX.sym('y2');
                        t = SX.sym('t');
                        x = [y1;y2;t];
                        c = [y1-nu;y2];

                        eps = 1e-12;
                        r=2*sin(14*t);
                        u1 = 0; % for y1 < 0
                        u2 = c_erthq*(abs(y1-nu)+eps)^(3/2) + 1.98*sqrt(2*c_erthq)*(abs(y1-nu)+eps)^(1/4)*y2; % for y1>0, y2>0
                        u3 = c_erthq*(abs(y1-nu)+eps)^(3/2); % for y1>0, y2<0

                        f0 = [y2;(-4.1*y2-k*y1-r)/2;1];
                        f1 = f0-[0;u1/2;0];
                        f2 = f0-[0;u2/2;0];
                        f3 = f0-[0;u3/2;0];

                        F = [f1 f2 f3];
                        S = [-1 0;...
                            1 1;...
                            1 -1];


                        F = [f1 f1 f2 f3];
                        S = [-1 1;...
                            -1 -1;...
                            1 1;...
                            1 -1];

                        %% populate model
                        problem_options.T = 3/500;
                        model.x = x;
                        model.c = c;
                        model.S = S;
                        model.F = F;
                        %% Solve OCP
                        % TODO annoying serialization issues
                        %mpcc = NosnocMPCC(problem_options, model.dims, model);
                        problem_options.preprocess();
                        model.verify_and_backfill(problem_options);
                        model.generate_variables(problem_options);
                        model.generate_equations(problem_options);
                        filename = generate_problem_name(model, problem_options, idx);
                        save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');
                        index = index+1;
                    end
                end
            end
        end
    end
end
