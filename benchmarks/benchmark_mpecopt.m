close all;
clear all;

problem_lists = readlines('problem_lists/nosbench-rs');

so1 = nosnoc.reg_homotopy.Options();
so1.solver_name = 'so1';
so1.homotopy_steering_strategy = "DIRECT";
so1.decreasing_s_elastic_upper_bound = true;
so1.complementarity_tol = 1e-7;
so1.opts_casadi_nlp.ipopt.max_iter = 5e3;
so1.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
so1.sigma_0 = 1e0;
so1.homotopy_update_slope = 0.1;
so1.print_level = 3;
so1.timeout_wall = 3600;

mpecopt_opts = mpecopt.Options();

sopts = {so1,mpecopt_opts};

[job,tasks] = run_benchmark(problem_lists, sopts, @solve_with_nosnoc, true, "MPECOPT_BENCH");

function stats = solve_with_nosnoc(json, options, problem_name)
    mpcc = vdx.problems.Mpcc.from_json(json);
    if metaclass(options) == ?mpecopt.Options
        mpcc.create_solver(options, 'mpecopt');
    else
        mpcc.create_solver(options);
    end
    stats = mpcc.solve();
end
