clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [2];
N_FE = [2];
DCS_MODES = [DcsMode.Step, DcsMode.Stewart];
INITIAL_POINTS = {[3;0;0],
    [1.2992;1.2595;26.1000]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for lift=[false, true]
        for idx=1:length(INITIAL_POINTS)
            for n_s=N_S
                for N_fe=N_FE
                    for dcs_mode=DCS_MODES
                        problem_options = nosnoc.Options();
                        model = NosnocModel();
                        model.model_name = ['986OM'];
                        problem_options.n_s = n_s;                            
                        problem_options.cross_comp_mode = cross_comp_mode;
                        problem_options.dcs_mode = dcs_mode;
                        problem_options.N_finite_elements = N_fe;
                        problem_options.lift_complementarities = lift;
                        model.x0 = INITIAL_POINTS{idx};

                        problem_options.use_fesd = 1;       % switch detection method on/off

                        %% Time settings
                        y = SX.sym('y',2);
                        t = SX.sym('t',1);
                        x = [y;t];
                        S = [1;-1];
                        v=cos(t)+0.7;
                        c = y(2)-v;
                        k=1.0;
                        r=0.2;
                        a0=1.0;
                        w=0.7;
                        Fc=0.4;
                        f0=[y(2);-k*y(1)-2*r*y(2)+a0*cos(w*t);1];
                        f1= f0+[0;-Fc;0];
                        f2= f0+[0; Fc;0];
                        F = [f1 f2];

                        %% populate model
                        problem_options.T = 30/200;
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
                        filename = generate_problem_name(model, problem_options, idx)
                        save(['../../level', num2str(LEVEL) ,'/', char(filename), '.mat'], 'model', 'problem_options');
                        index = index+1;
                    end
                end
            end
        end
    end
end
