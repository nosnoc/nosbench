function results = run_benchmark(problem_list, options_list, solver_fun, use_vdx)
    arguments
        problem_list(1,:) string
        options_list(1,:) cell
        solver_fun(1,1) function_handle
        use_vdx(1,1) logical = true
    end
    n_problems = length(problem_list);
    n_options = length(options_list);

    n_total = n_problems*n_options;

    instances = {};
    
    for ii=1:n_problems
        for jj=1:n_options
            instance.problem = problem_list(ii);
            instance.options = options_list{jj};
            instances{ii,jj} = instance;
        end
    end

    instances = reshape(instances, [n_total,1]);
    instances = instances(randperm(n_total));
    results = cell(n_total,1);
    [filepath,~,~] = fileparts(mfilename('fullpath'));
    for ii=1:n_total
        instance = instances{ii};
        if use_vdx
            json = fileread([char(filepath) '/../problems/vdx/' char(instance.problem), '.json']);
            problem = vdx.problems.Mpcc.from_json(json);
        else
            json = fileread([char(filepath) '/../problems/casadi/' char(instance.problem), '.json']);
            raw_mpcc = jsondecode();
            mpcc.w = SX.deserialize(raw_mpcc.w);
            mpcc.p = SX.deserialize(raw_mpcc.p);
            mpcc.f_fun = Function.deserialize(raw_mpcc.f_fun);
            mpcc.f = mpcc.f_fun(mpcc.w, mpcc.p);
            mpcc.g_fun = Function.deserialize(raw_mpcc.g_fun);
            mpcc.g = mpcc.f_fun(mpcc.w, mpcc.p);
            mpcc.G_fun = Function.deserialize(raw_mpcc.G_fun);
            mpcc.G = mpcc.G_fun(mpcc.w, mpcc.p);
            mpcc.H_fun = Function.deserialize(raw_mpcc.H_fun);
            mpcc.H = mpcc.H_fun(mpcc.w, mpcc.p);
            problem = mpcc;
        end
        
        stats = solver_fun(problem, instance.options);
        result.stats = stats;
        result.options = instance.options;
        result.problem_name = instance.problem;
        results{ii} = result;
    end
end
