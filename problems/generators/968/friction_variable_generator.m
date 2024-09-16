clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [2];
N_FE = [2];
DCS_MODES = [DcsMode.Heaviside, DcsMode.Stewart];
INITIAL_POINTS = {[-2;3;0;0],
    [-0.0075;0.7870;3.0307;-3.4507],
    [1.9337;-1.8933;0.0055;-0.9974]};

% TODO(anton) please make an automatic naming generator
index = 1;

for cross_comp_mode=CROSS_COMP_MODES
    for idx=1:length(INITIAL_POINTS)
        for n_s=N_S
            for N_fe=N_FE
                for dcs_mode=DCS_MODES
                    problem_options = nosnoc.Options();
                    model = nosnoc.model.Pss();
                    model_name = ['968FV'];
                    problem_options.n_s = n_s;                            
                    problem_options.cross_comp_mode = cross_comp_mode;
                    problem_options.dcs_mode = dcs_mode;
                    problem_options.N_finite_elements = N_fe;

                    model.x0 = INITIAL_POINTS{idx};
                    problem_options.use_fesd = 1;       % switch detection method on/off
                    problem_options.rk_scheme = 'GAUSS_LEGENDRE';
                    
                    %% Time settings
                    y = SX.sym('y',4);
                    x = y;
                    c1 = [y(1);y(3)];
                    c2 = [y(2);y(4)];
                    S1 = [1 1;...
                        1 -1;
                        -1  1;...
                        -1 -1];
                    S2 = S1;

                    mu1_pos=1.0;
                    mu1_neg=0.6;
                    mu2_pos=0.2;
                    mu2_neg=0.5;

                    E=1.0;
                    f0 =[y(3:4);...
                        -E*(y(1)-y(2)); ...
                        -E*(y(2)-y(1))];
                    f11 = [0;0;-mu1_pos;0];
                    f12 = [0;0; mu1_pos;0];
                    f13 = [0;0;-mu1_neg;0];
                    f14 = [0;0; mu1_neg;0];

                    f21 = [0;0;0;-mu2_pos];
                    f22 = [0;0;0; mu2_pos];
                    f23 = [0;0;0;-mu2_neg];
                    f24 = [0;0;0; mu2_neg];

                    F1 = [f0+f11 f0+f12 f0+f13 f0+f14];
                    F2 = [f21 f22 f23 f24];

                    %% populate model
                    problem_options.T = 5/500;
                    model.x = x;
                    model.c = {c1,c2};
                    model.S = {S1,S2};
                    model.F = {F1,F2};

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
