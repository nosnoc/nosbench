close all
clear all

convex = 1;
if convex
    bench_name = "QPCC_CONVEX"
    qpcc_names = readlines("problist_convex");
else
    bench_name = "QPCC"
    qpcc_names = readlines("problist");
end
save_data = 0;

ccopt_options = nosnoc.ccopt.Options(); % ccopt options 
ccopt_options.opts_madnlp.linear_solver = 'Ma27Solver';
ccopt_options.opts_ccopt.relaxation_update.TYPE = 'RolloffRelaxationUpdate';
ccopt_options.opts_ccopt.relaxation_update.rolloff_slope = 2.0;
ccopt_options.opts_ccopt.relaxation_update.rolloff_point = 1e-6;
ccopt_options.opts_ccopt.relaxation_update.sigma_min = 1e-8;
ccopt_options.opts_madnlp.tol=1e-8;
ccopt_options.opts_ccopt.q_regularization = 'critical_rho';
ccopt_options.opts_ccopt.critical_rho_factor = 0.999;
ccopt_options.opts_ccopt.endgame_strategy.TYPE = 'RelaxLBEndgameStrategy';
ccopt_options.opts_ccopt.endgame_threshold = 1e-6;

solver_options  = nosnoc.reg_homotopy.Options();
solver_options.N_homotopy = 8;
solver_options.homotopy_update_slope = 0.1;
solver_options.complementarity_tol = 1e-6;
solver_options.sigma_0 = 1;
solver_options.relaxation_strategy = "SCHOLTES_INEQ";
solver_options.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
solver_options.print_level = 3;
solver_options.opts_casadi_nlp.ipopt.hsllib = '/home/anton/tools/HSL_jll.jl-2023.11.7/override/lib/x86_64-linux-gnu-libgfortran5/libhsl.so';

gurobi_miqp_options = nosnoc.qpec.GurobiOptions();
gurobi_miqp_options.gurobi_params.NodeLimit = 1e4;
gurobi_miqp_options.recover_duals = false;
gurobi_miqp_options.resolve_qp_for_duals = false;

gurobi_sos1_options = nosnoc.qpec.GurobiOptions();
gurobi_sos1_options.method = "sos1";
gurobi_sos1_options.recover_duals = false;
gurobi_sos1_options.resolve_qp_for_duals = false;
gurobi_sos1_options.gurobi_params.OutputFlag = 0;
gurobi_sos1_options.gurobi_params.NodeLimit = 1e4;
gurobi_sos1_options.gurobi_params.Presolve = -1;
gurobi_sos1_options.gurobi_params.MIPGap = 1e-4;
gurobi_sos1_options.gurobi_params.FeasibilityTol = 1e-6;
gurobi_sos1_options.gurobi_params.NumericFocus = 2;
gurobi_sos1_options.gurobi_params.ThreadLimit = 1;


gurobi_reg_options = nosnoc.qpec.GurobiOptions();
gurobi_reg_options.method = "reg";

lcqpow_options = LCQPow_options();
lcqpow_options.complementarityTolerance = 1e-6;
lcqpow_options.stationarityTolerance = 1e-6;
lcqpow_options.printLevel = 1;
lcqpow_options.qpSolver = 1;

solvers = {
    ccopt_options,'ccopt', "CCOpt Relaxation";
    solver_options, 'reg_homotopy', "IPOPT Homotopy";
    gurobi_sos1_options, 'gurobi', "Gurobi SOS1";
    lcqpow_options, 'lcqpow', "LCQPow";
    };

data = {};

for ii=[1]%[1:size(solvers,1)]
    solver = solvers{ii,1};
    solver_plugin = solvers{ii,2};
    solver_name = solvers{ii,3};
    for jj=1:length(qpcc_names)
        qpcc_name = qpcc_names(jj);
        [qpcc,name] = load_qpec(strcat("data/",qpcc_name));
        qpcc.create_qpec_solver(solver, solver_plugin);
        stats = qpcc.solve();
        switch solver_plugin
            case 'ccopt'
                wall_time = qpcc.stats.ccopt.total_wall_time;
                success = qpcc.stats.success;
                obj = qpcc.result_qpec.f;
            case 'gurobi'
                wall_time = qpcc.stats.runtime;
                success = qpcc.stats.success;
                obj = qpcc.result_qpec.obj;
            case 'reg_homotopy'
                wall_time = qpcc.stats.wall_time_total;
                success = qpcc.stats.success;
                obj = qpcc.result_qpec.f;
            case 'lcqpow'
                wall_time = qpcc.stats.elapsed_time;
                success = qpcc.stats.exit_flag == 0;
                obj = qpcc.result_qpec.f;
        end
        data{end+1} = {solver_name, qpcc_name, success, wall_time, obj};
    end
end
data_table = cell2table(vertcat(data{:}),"VariableNames",["solver_name", "problem_name", "success", "wall_time_total", "objective"]);

best_time = pivot(data_table, Rows=["problem_name"], Columns=["success"], DataVariable=["wall_time_total"], Method="min");
best_time_dict = dictionary(best_time{:, "problem_name"},best_time{:, "true"});
data_table.ratio_best_time = (data_table.wall_time_total./best_time_dict(data_table.problem_name));

best_objective = pivot(data_table, Rows=["problem_name"], Columns=["success"], DataVariable=["objective"], Method="min");
best_objective_dict = dictionary(best_time{:, "problem_name"},best_objective{:, "true"});
data_table.ratio_best_objective = (data_table.objective./best_objective_dict(data_table.problem_name));

if save_data
    save([char(bench_name) '_results.mat'], 'data_table');
end