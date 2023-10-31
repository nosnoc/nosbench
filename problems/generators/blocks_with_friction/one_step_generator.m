clear all
close all
import casadi.*

LEVEL = 1;

CROSS_COMP_MODES = [1,3,4,7];
N_S = [2];
N_FE = [3];
DCS_MODES = [DcsMode.Step, DcsMode.Stewart];
INITIAL_POINTS = {[-1.0991; 1.0902;  -0.7588; -0.4115; 0.2943; 2.3575; 0.1411],
    [0.3491; -0.0470; -0.3207; 0.1599; -0.2828; 2.1859; 2.1176],
    [-1;1;-1;-1;1;1;0]};

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
                        model.model_name = ['FBS1S'];
                        problem_options.T = 12/85;
                        problem_options.n_s = n_s;                            
                        problem_options.irk_scheme = IRKSchemes.RADAU_IIA;
                        problem_options.use_fesd = 1;
                        problem_options.cross_comp_mode = cross_comp_mode;
                        problem_options.dcs_mode = dcs_mode;
                        problem_options.N_finite_elements = N_fe;
                        problem_options.lift_complementarities = lift;
                        %% Initial value
                        model.x0 = INITIAL_POINTS{idx};
                        model.u0 = 0; % guess for control variables

                        %% Variable defintion
                        % differential states
                        q1 = SX.sym('q1');
                        q2 = SX.sym('q2');
                        q3 = SX.sym('q3');
                        v1 = SX.sym('v1');
                        v2 = SX.sym('v2');
                        v3 = SX.sym('v3');
                        t = SX.sym('t');

                        q = [q1;q2;q3];
                        v = [v1;v2;v3];
                        model.x = [q;v;t];

                        %% Switching Functions
                        % every constraint function corresponds to a simplex (note that the c_i might be vector valued)
                        c1 = v1;
                        c2 = v2;
                        c3 = v3;
                        % sign matrix for the modes
                        S1 = [1;-1];
                        S2 = [1;-1];
                        S3 = [1;-1];
                        % discrimnant functions
                        model.S = {S1,S2,S3};
                        model.c = {c1,c2,c3};


                        %% Modes of the ODEs layers (for all  i = 1,...,n_sys);
                        % part independet of the nonsmoothness
                        F_external = 0; % external force, e.g., control
                        F_input = 10; % variable force exicting
                        f_base = [v1;...
                            v2;...
                            v3;...
                            (-q1)+(q2-q1)-v1;...
                            (q1-q2)+(q3-q2)-v2;...
                            (q2-q3)-v3+F_input*cos(pi*t);...
                                 ]/6;

                        f_base = [v1;...
                            v2;...
                            v3;...
                            (-q1)+(q2-q1)-v1;...
                            (q1-q2)+(q3-q2)-v2;...
                            (q2-q3)-v3+F_external+F_input*(1*0+1*cos(pi*t));...
                            1];
                        %
                        % for c1, h1,
                        f_11 = f_base+[0;0;0;-0.3;0;0;0];
                        f_12 = f_base+[0;0;0;+0.3;0;0;0];
                        % for c2, h2
                        f_21 = [0;0;0;0;-0.3;0;0];
                        f_22 = [0;0;0;0;0.3;0;0];
                        % for c3, h3
                        f_31 = [0;0;0;0;0;-0.3;0];
                        f_32 = [0;0;0;0;0;0.3;0];
                        % unfold_struct(model,'base');
                        % in matrix form
                        F1 = [f_11 f_12];
                        F2 = [f_21 f_22];
                        F3 = [f_31 f_32];

                        model.F = {F1 F2 F3};
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
