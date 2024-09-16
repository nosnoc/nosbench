clear all
close all
import casadi.*

BENCH_NAME = ['RELAXATION_BENCH_', char(datetime('today','Format','yyyy-MM-dd'))];

C_FUN=[CFunctionType.SCHOLTES,...
    CFunctionType.FISCHER_BURMEISTER,...
    CFunctionType.NATURAL_RESIDUAL,...
    CFunctionType.CHEN_CHEN_KANZOW,...
    CFunctionType.STEFFENSEN_ULBRICH,...
    CFunctionType.STEFFENSEN_ULBRICH_POLY,...
    CFunctionType.KANZOW_SCHWARTZ,...
    CFunctionType.KADRANI,...
    CFunctionType.LIN_FUKUSHIMA];
ELASTICITY = [ElasticityMode.NONE, ElasticityMode.ELL_INF, ElasticityMode.ELL_1];
solvers = {};
for c_fun=C_FUN
    for elasticity=ELASTICITY
        solver_options = NosnocSolverOptions();
        solver_options.psi_fun_type = c_fun;
        solver_options.elasticity_mode = elasticity;
        solver_options.comp_tol = 1e-7;
        solver_options.opts_casadi_nlp.ipopt.max_iter = 5e3;
        solver_options.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
        solver_options.sigma_0 = 1e0;
        solver_options.homotopy_update_slope = 0.1;
        solver_options.solver_name = [char(c_fun), '_', char(elasticity)];
        solver_options.print_level = 3;
	solver_options.timeout_wall = 300;
        solver_options.normalize_homotopy_update = true;

        if c_fun == CFunctionType.KADRANI || c_fun == CFunctionType.KANZOW_SCHWARTZ
            solver_options.normalize_homotopy_update = false;
        end
        if c_fun == CFunctionType.LIN_FUKUSHIMA && elasticity == ElasticityMode.ELL_1
            solver_options.normalize_homotopy_update = false;
        end
        if c_fun == CFunctionType.STEFFENSEN_ULBRICH && elasticity == ElasticityMode.ELL_1
            solver_options.normalize_homotopy_update = false;
        end
        if c_fun == CFunctionType.STEFFENSEN_ULBRICH_POLY && elasticity ~= ElasticityMode.NONE
            solver_options.normalize_homotopy_update = false;
        end
        solvers = [solvers,solver_options];
    end
end

% add direct solver
solver_options = NosnocSolverOptions();
solver_options.sigma_0=0;
solver_options.N_homotopy=1;
solver_options.opts_casadi_nlp.ipopt.max_iter = 5e3;
solver_options.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
solver_options.print_level = 3;
solver_options.timeout_wall = 3600;
solver_options.comp_tol = 1e-7;
solver_options.solver_name = ['direct'];
solvers = [solvers,solver_options];

% add ell_1_penalty solver
solver_options = NosnocSolverOptions();
solver_options.opts_casadi_nlp.ipopt.max_iter = 5e3;
solver_options.opts_casadi_nlp.ipopt.linear_solver = 'ma27';
solver_options.print_level = 3;
solver_options.timeout_wall = 3600;
solver_options.sigma_0 = 1e0;
solver_options.homotopy_update_slope = 0.1;
solver_options.mpcc_mode = MpccMode.ell_1_penalty;
solver_options.objective_scaling_direct = 0;
solver_options.comp_tol = 1e-7;
solver_options.solver_name = ['ell_1_penalty'];
solvers = [solvers,solver_options];

problems = readlines("problem_lists/all_problems");

results = run_benchmark(problems, solvers, @solve_with_homotopy_solver);
