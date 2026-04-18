instance = "TFBIB_001_040_004_2_RIIA_STEWART_7_ELC";

%%
ccopt_rolloff = nosnoc.ccopt.Options();
ccopt_rolloff.solver_name = 'CCOpt Rolloff';
ccopt_rolloff.opts_madnlp.linear_solver = 'Ma27Solver';
ccopt_rolloff.opts_madnlp.barrier.TYPE = 'MonotoneUpdate';
ccopt_rolloff.opts_madnlp.barrier.mu_min = 1e-9;
ccopt_rolloff.opts_ccopt.relaxation_update.TYPE = 'RolloffRelaxationUpdate';
ccopt_rolloff.opts_ccopt.relaxation_update.rolloff_slope = 2.0;
ccopt_rolloff.opts_ccopt.relaxation_update.rolloff_point = 1e-6;
ccopt_rolloff.opts_ccopt.relaxation_update.rolloff_max = 1.0;
ccopt_rolloff.opts_ccopt.relaxation_update.sigma_min = 1e-8;
ccopt_rolloff.opts_madnlp.tol=1e-8;
ccopt_rolloff.opts_madnlp.max_iter=3000;
ccopt_rolloff.opts_madnlp.disable_garbage_collector = false;
%ccopt_rolloff.opts_ccopt.q_regularization = 'eigenvalue_decomposition';
ccopt_rolloff.opts_ccopt.min_eig_value= 1e-8;
%ccopt_rolloff.opts_ccopt.q_regularization = 'critical_rho';
ccopt_rolloff.opts_ccopt.critical_rho_factor = 0.999;
ccopt_rolloff.opts_ccopt.endgame_strategy = 'RelaxLBEndgameStrategy';
ccopt_options.opts_ccopt.endgame_threshold = 1e-5;

json = fileread(strcat("vdx/", instance, ".json"));
mpcc = vdx.problems.Mpcc.from_json(json);
mpcc.create_solver(ccopt_rolloff, 'ccopt');
stats = mpcc.solve()

%%
ccopt_lb = nosnoc.ccopt.Options();
ccopt_lb.solver_name = 'CCOpt relax lb';
ccopt_lb.opts_madnlp.linear_solver = 'Ma27Solver';
ccopt_lb.opts_madnlp.barrier.TYPE = 'MonotoneUpdate';
ccopt_lb.opts_madnlp.barrier.mu_min = 1e-9;
ccopt_lb.opts_ccopt.relaxation_update.TYPE = 'RelaxLBUpdate';
ccopt_lb.opts_ccopt.relaxation_update.relax_threshold = 1e-4;
ccopt_lb.opts_madnlp.tol=1e-7;
ccopt_lb.opts_madnlp.max_iter=3000;
ccopt_lb.opts_madnlp.disable_garbage_collector = true;
%ccopt_rolloff.opts_ccopt.q_regularization = 'eigenvalue_decomposition';

json = fileread(strcat("vdx/", instance, ".json"));
mpcc = vdx.problems.Mpcc.from_json(json);
mpcc.create_solver(ccopt_lb, 'ccopt');
stats = mpcc.solve()
%%
ccopt_qf = nosnoc.ccopt.Options();
ccopt_qf.solver_name = 'CCOpt Quality Function';
ccopt_qf.opts_madnlp.linear_solver = 'Ma27Solver';
ccopt_qf.opts_madnlp.barrier.TYPE = 'QualityFunctionUpdate';
ccopt_qf.opts_madnlp.barrier.mu_min = 1e-9;
ccopt_qf.opts_madnlp.barrier.max_gs_iter = 16;
ccopt_qf.opts_madnlp.barrier.mu_max = 1e2;
ccopt_qf.opts_ccopt.relaxation_update.TYPE = 'ProportionalRelaxationUpdate';
ccopt_qf.opts_ccopt.relaxation_update.sigma_min = 1e-9;
ccopt_qf.opts_madnlp.tol=1e-7;
ccopt_qf.opts_madnlp.max_iter=3000;

json = fileread(strcat("vdx/", instance, ".json"));
mpcc = vdx.problems.Mpcc.from_json(json);
mpcc.create_solver(ccopt_qf, 'ccopt');
stats = mpcc.solve()

%%
ccopt_monotone = nosnoc.ccopt.Options();
ccopt_monotone.solver_name = 'CCOpt Monotone';
ccopt_monotone.opts_madnlp.linear_solver = 'Ma27Solver';
ccopt_monotone.opts_madnlp.barrier.TYPE = 'MonotoneUpdate';
ccopt_monotone.opts_madnlp.barrier.mu_min = 1e-9;
ccopt_monotone.opts_ccopt.relaxation_update.TYPE = 'ProportionalRelaxationUpdate';
ccopt_monotone.opts_ccopt.relaxation_update.sigma_min = 1e-7;
ccopt_monotone.opts_madnlp.tol=1e-7;
ccopt_monotone.opts_madnlp.max_iter=3000;

json = fileread(strcat("vdx/", instance, ".json"));
mpcc = vdx.problems.Mpcc.from_json(json);
mpcc.create_solver(ccopt_monotone, 'ccopt');
stats = mpcc.solve()
%%
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
so1.print_level = 0;
so1.timeout_wall = 3600;
so1.normalize_homotopy_update = true;

json = fileread(strcat("vdx/", instance, ".json"));
mpcc = vdx.problems.Mpcc.from_json(json);
mpcc.create_solver(so1);
stats = mpcc.solve()