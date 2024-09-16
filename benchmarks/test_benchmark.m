close all;
clear all;

problem_lists = readlines('problem_lists/test_list');

so1 = nosnoc.solver.Options();
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
so1.normalize_homotopy_update = true;

so2 = nosnoc.solver.Options();
so2.solver_name = 'so2';
so2.homotopy_steering_strategy = "ELL_INF";
so2.decreasing_s_elastic_upper_bound = true;
so2.complementarity_tol = 1e-7;
so2.opts_casadi_nlp.ipopt.max_iter = 5e3;
so2.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
so2.sigma_0 = 1e0;
so2.homotopy_update_slope = 0.1;
so2.print_level = 3;
so2.timeout_wall = 3600;
so2.normalize_homotopy_update = true;

so3 = nosnoc.solver.Options();
so3.solver_name = 'so3';
so3.homotopy_steering_strategy = "ELL_1";
so3.decreasing_s_elastic_upper_bound = true;
so3.complementarity_tol = 1e-7;
so3.opts_casadi_nlp.ipopt.max_iter = 5e3;
so3.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
so3.sigma_0 = 1e0;
so3.homotopy_update_slope = 0.1;
so3.print_level = 3;
so3.timeout_wall = 3600;
so3.normalize_homotopy_update = true;

sopts = {so1,so2,so3};

res = run_benchmark(problem_lists, sopts, @solve_with_nosnoc, true, "TEST_BENCH");

function stats = solve_with_nosnoc(mpcc, options)
    mpcc.create_solver(options);
    stats = mpcc.solve();
end
