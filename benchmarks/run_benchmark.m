function [results,jobs] = run_benchmark(problem_list, options_list, solver_fun, use_vdx, bench_name)
    arguments
        problem_list(1,:) string
        options_list(1,:) cell
        solver_fun(1,1) function_handle
        use_vdx(1,1) logical = true
        bench_name string = []
    end
    % process bench name
    % TODO(probably a better log file is needed)
    if ~isempty(bench_name)
        BENCH_NAME = [char(bench_name), '_', char(datetime('today','Format','yyyy-MM-dd'))];
        results_dir = ['~/results/NOSBENCH_results/', BENCH_NAME];
        mkdir(results_dir);
    end
    n_problems = length(problem_list);
    n_options = length(options_list);

    n_total = n_problems*n_options;

    instances = {};
    
    for ii=1:n_problems
        for jj=1:n_options
            instances{ii,jj} = {problem_list(ii),options_list{jj}};
        end
    end

    instances = reshape(instances, [n_total,1]);
    instances = instances(randperm(n_total));
    results = cell(n_total,1);
    [filepath,~,~] = fileparts(mfilename('fullpath'));
    for ii=1:n_total
        ii
        instance = instances{ii};
        problem = instance{1};
        options = instance{2};
        if use_vdx
            [~,json] = system(['tar -xf ' fullfile(char(filepath), '../problems/vdx.tar.xz') ' vdx/' char(problem) '.json --to-stdout']);
            
            mpcc = vdx.problems.Mpcc.from_json(json);
        else
            [~,json] = system(['tar -xf ' fullfile(char(filepath), '../problems/casadi.tar.xz') ' vdx/', char(problem) '.json --to-stdout']);
            raw_mpcc = jsondecode(json);
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
        end

        % TODO do restart again
        % if ~isempty(bench_name)
        %     outfile = ['~/results/NOSBENCH_results/', BENCH_NAME, '/', char(instance.options.solver_name), '_', char(instance.problem), '.mat'];
        %     if isfile(outfile)
        %         disp(['Skipping: ' outfile])
        %         continue;
        %     end
        % end
        job = batch(solver_fun, 1, {json, options}, 'CaptureDiary', true, 'AutoAttachFiles', false);
        jobs(ii) = job;
    end
    results = [];
end
