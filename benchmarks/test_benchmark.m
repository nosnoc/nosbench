close all;
clear all;

problem_lists = ["CPWF_002_057_002_2_RIIA_STEWART_7_FIL","MWFOCP_001_053_003_2_RIIA_HEAVISIDE_7_FIL"];

so1 = nosnoc.solver.Options();
so1.homotopy_steering_strategy = "DIRECT";
so1.decreasing_s_elastic_upper_bound = true;
so1.opts_casadi_nlp.ipopt.max_iter = 1000;
so2 = nosnoc.solver.Options();
so2.homotopy_steering_strategy = "ELL_INF";
so2.decreasing_s_elastic_upper_bound = true;
so2.opts_casadi_nlp.ipopt.max_iter = 1000;
so3 = nosnoc.solver.Options();
so3.homotopy_steering_strategy = "ELL_1";
so3.decreasing_s_elastic_upper_bound = true;
so3.opts_casadi_nlp.ipopt.max_iter = 1000;

sopts = {so1,so2,so3};


res = run_benchmark(problem_lists, sopts, @solve_with_nosnoc);


function stats = solve_with_nosnoc(mpcc, options)
    mpcc.create_solver(options);
    stats = mpcc.solve();
end
