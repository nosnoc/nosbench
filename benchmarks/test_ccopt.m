close all;
clear all;

problem_lists = readlines('problem_lists/test_list');

so1 = nosnoc.reg_homotopy.Options();
so1.solver_name = 'Scholtes Homotopy';
so1.homotopy_steering_strategy = "DIRECT";
so1.decreasing_s_elastic_upper_bound = true;
so1.complementarity_tol = 1e-7;
so1.opts_casadi_nlp.ipopt.max_iter = 5e3;
so1.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
so1.opts_casadi_nlp.ipopt.hsllib = '/home/anton/tools/HSL_jll.jl-2023.11.7/override/lib/x86_64-linux-gnu-libgfortran5/libhsl.so';
so1.sigma_0 = 1e0;
so1.homotopy_update_slope = 0.1;
so1.print_level = 3;
so1.timeout_wall = 3600;
so1.normalize_homotopy_update = true;

so2 = nosnoc.ccopt.Options();
so2.solver_name = 'CCOpt';
so2.opts_madnlp.linear_solver = 'Ma27Solver';
so2.opts_madnlp.barrier.TYPE = 'QualityFunctionUpdate';

so3 = nosnoc.reg_homotopy.Options();
so3.solver_name = 'reg_homotopy ell infinity';
so3.homotopy_steering_strategy = "ELL_INF";
so3.decreasing_s_elastic_upper_bound = true;
so3.complementarity_tol = 1e-7;
so3.opts_casadi_nlp.ipopt.max_iter = 5e3;
so3.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
so3.opts_casadi_nlp.ipopt.hsllib = '/home/anton/tools/HSL_jll.jl-2023.11.7/override/lib/x86_64-linux-gnu-libgfortran5/libhsl.so';
so3.sigma_0 = 1e0;
so3.homotopy_update_slope = 0.1;
so3.print_level = 3;
so3.timeout_wall = 3600;
so3.normalize_homotopy_update = true;

sopts = {so1,so2,so3};

[job,tasks] = run_benchmark(problem_lists, sopts, @solve_with_nosnoc, true, "CCOPT_BENCH",false,false,true);

function stats = solve_with_nosnoc(json, options, problem_name)
    mpcc = vdx.problems.Mpcc.from_json(json);
    if metaclass(options) == ?nosnoc.ccopt.Options
        mpcc.create_solver(options,'ccopt');
        mpcc.solve();% to trigger precompile
    else
        mpcc.create_solver(options);
    end
    stats = mpcc.solve();
end