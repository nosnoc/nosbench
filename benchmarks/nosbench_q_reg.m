close all;
clear all;

problem_lists = readlines('problem_lists/nosbench-ccopt-rs');

so1 = nosnoc.reg_homotopy.Options();
so1.solver_name = 'Scholtes Homotopy';
so1.homotopy_steering_strategy = "DIRECT";
so1.decreasing_s_elastic_upper_bound = true;
so1.complementarity_tol = 1e-7;
so1.opts_casadi_nlp.ipopt.tol = 1e-7;
so1.opts_casadi_nlp.ipopt.dual_inf_tol = 1e-7;
so1.opts_casadi_nlp.ipopt.dual_inf_tol = 1e-7;
so1.opts_casadi_nlp.ipopt.compl_inf_tol = 1e-7;
so1.opts_casadi_nlp.ipopt.max_iter = 3e3;
so1.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
so1.opts_casadi_nlp.ipopt.hsllib = '/home/anton/tools/HSL_jll.jl-2023.11.7/override/lib/x86_64-linux-gnu-libgfortran5/libhsl.so';
so1.sigma_0 = 1e0;
so1.homotopy_update_slope = 0.1;
so1.print_level = 3;
so1.timeout_wall = 600;
so1.normalize_homotopy_update = true;

so2 = nosnoc.ccopt.Options();
so2.solver_name = 'CCOpt Rolloff';
so2.opts_madnlp.linear_solver = 'Ma27Solver';
so2.opts_madnlp.barrier.TYPE = 'MonotoneUpdate';
so2.opts_madnlp.barrier.mu_min = 1e-9;
so2.opts_ccopt.relaxation_update.TYPE = 'RolloffRelaxationUpdate';
so2.opts_ccopt.relaxation_update.rolloff_slope = 2.0;
so2.opts_ccopt.relaxation_update.rolloff_point = 1e-6;
so2.opts_ccopt.relaxation_update.sigma_min = 1e-7;
so2.opts_madnlp.tol=1e-7;
so2.opts_madnlp.print_level = 6;
so2.opts_madnlp.max_iter=3000;
so2.opts_madnlp.disable_garbage_collector = true;

so3 = nosnoc.ccopt.Options();
so3.solver_name = 'CCOpt Rolloff B hat';
so3.opts_madnlp.linear_solver = 'Ma27Solver';
so3.opts_madnlp.barrier.TYPE = 'MonotoneUpdate';
so3.opts_madnlp.barrier.mu_min = 1e-9;
so3.opts_ccopt.relaxation_update.TYPE = 'RolloffRelaxationUpdate';
so3.opts_ccopt.relaxation_update.rolloff_slope = 2.0;
so3.opts_ccopt.relaxation_update.rolloff_point = 1e-6;
so3.opts_ccopt.relaxation_update.sigma_min = 1e-7;
so3.opts_madnlp.tol=1e-7;
so3.opts_madnlp.print_level = 6;
so3.opts_madnlp.max_iter=3000;
so3.opts_madnlp.disable_garbage_collector = true;
so3.opts_ccopt.q_regularization = 'eigenvalue_decomposition';
so3.opts_ccopt.min_eig_value= 1e-8;

so4 = nosnoc.ccopt.Options();
so4.solver_name = 'CCOpt Rolloff B bar';
so4.opts_madnlp.linear_solver = 'Ma27Solver';
so4.opts_madnlp.barrier.TYPE = 'MonotoneUpdate';
so4.opts_madnlp.barrier.mu_min = 1e-9;
so4.opts_ccopt.relaxation_update.TYPE = 'RolloffRelaxationUpdate';
so4.opts_ccopt.relaxation_update.rolloff_slope = 2.0;
so4.opts_ccopt.relaxation_update.rolloff_point = 1e-6;
so4.opts_ccopt.relaxation_update.sigma_min = 1e-7;
so4.opts_madnlp.tol=1e-7;
so4.opts_madnlp.print_level = 6;
so4.opts_madnlp.max_iter=3000;
so4.opts_madnlp.disable_garbage_collector = true;
s04.opts_ccopt.q_regularization = 'critical_rho';
s04.opts_ccopt.critical_rho_factor = 0.999;

%sopts = {so1,so2,so3,so4};
sopts = {so2,so3,so4};

[job,tasks] = run_benchmark(problem_lists, sopts, @solve_with_nosnoc, true, "Q_REG_BENCH",false,false,true);

function stats = solve_with_nosnoc(json, options, problem_name)
    mpcc = vdx.problems.Mpcc.from_json(json);
    if metaclass(options) == ?nosnoc.ccopt.Options
        mpcc.create_solver(options,'ccopt');
        mpcc.solve();% to trigger precompile
    else
        mpcc.create_solver(options);
    end
    stats = mpcc.solve();
    stats.objective = mpcc.f_result;
    stats.problem_name = problem_name;
end