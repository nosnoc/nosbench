clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [3];
N_FE = [2];
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];
INITIAL_POINTS = {[3;0],
    [-2.1740;-0.3669],
    [-0.2028;0.0331]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for idx=1:length(INITIAL_POINTS)
        for n_s=N_S
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pss();
                    model_name = ['968FO'];
                    problem_options.n_s = n_s;                            
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.dcs_mode = dcs_mode;
                    problem_options.N_finite_elements = N_fe;

                    model.x0 = INITIAL_POINTS{idx};
                    
                    %% settings
                    % collocation settings
                    problem_options.use_fesd = 1;       % switch detection method on/off

                    %% Time settings
                    y1 = SX.sym('y1');
                    y2 = SX.sym('y2');
                    y = [y1;y2];
                    x = [y];

                    c = [y2];
                    Fc=0.4;
                    f1= [y(2);-y(1)+Fc];
                    f2= [y(2);-y(1)-Fc];

                    F = [f1 f2];
                    S = [-1;1];

                    %% populate model
                    problem_options.T = 20/100;
                    model.x = x;
                    model.c = c;
                    model.S = S;
                    model.F = F;

                    %% Generate problem
                    filename = generate_problem_name(model_name, model, problem_options, idx);
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
