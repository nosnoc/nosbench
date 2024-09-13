clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [2];
N_FE = [2];
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];
INITIAL_POINTS = {[3;0;0],
    [1.2992;1.2595;26.1000]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for idx=1:length(INITIAL_POINTS)
        for n_s=N_S
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pss();
                    model_name = ['968OM'];
                    problem_options.n_s = n_s;                            
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.dcs_mode = dcs_mode;
                    problem_options.N_finite_elements = N_fe;

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
                    fprintf(fid, '%s', casadi_json);
                    fclose(fid);
                    index = index+1;
                end
            end
        end
    end
end
