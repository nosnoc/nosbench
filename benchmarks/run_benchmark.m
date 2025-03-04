function [job,tasks] = run_benchmark(problem_list, options_list, solver_fun, use_vdx, bench_name, extract_problems, delete_problems, full_rerun)
    arguments
        problem_list(1,:) string
        options_list(1,:) cell
        solver_fun(1,1) function_handle
        use_vdx(1,1) logical = true
        bench_name string = 'NONAME_BENCH'
        extract_problems = false
        delete_problems = false
        full_rerun = false
    end
    % process bench name
    % TODO(probably a better log file is needed)
    BENCH_NAME = [char(bench_name), '_', char(datetime('today','Format','yyyy-MM-dd'))];
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
    if extract_problems
        if use_vdx
            system(['tar -xf ' fullfile(char(filepath), '../problems/vdx.tar.xz')]);
            %mpcc = vdx.problems.Mpcc.from_json(json);
        else
            system(['tar -xf ' fullfile(char(filepath), '../problems/casadi.tar.xz')]);
            % raw_mpcc = jsondecode(json);
            % mpcc.w = SX.deserialize(raw_mpcc.w);
            % mpcc.p = SX.deserialize(raw_mpcc.p);
            % mpcc.f_fun = Function.deserialize(raw_mpcc.f_fun);
            % mpcc.f = mpcc.f_fun(mpcc.w, mpcc.p);
            % mpcc.g_fun = Function.deserialize(raw_mpcc.g_fun);
            % mpcc.g = mpcc.f_fun(mpcc.w, mpcc.p);
            % mpcc.G_fun = Function.deserialize(raw_mpcc.G_fun);
            % mpcc.G = mpcc.G_fun(mpcc.w, mpcc.p);
            % mpcc.H_fun = Function.deserialize(raw_mpcc.H_fun);
            % mpcc.H = mpcc.H_fun(mpcc.w, mpcc.p);
        end
    end

    % Search for existing job:
    c = parcluster;
    oldjob = c.findJob(Tag=BENCH_NAME);

    if full_rerun
        delete(oldjob);
        oldjob = [];
    end
    
    if ~isempty(oldjob)
        % TODO ideally we recreate the newest one.
        job = recreate(oldjob(end), 'TaskState', 'failed');
    else
        job = createJob(c);
        job.UserData = struct();
        
        job.AutoAttachFiles = false;
        job.Tag = BENCH_NAME;
        for ii=1:n_total
            ii
            instance = instances{ii};
            problem = instance{1};
            options = instance{2};
            if use_vdx
                json = fileread([char(filepath) '/vdx/' char(problem), '.json']);
            else
                json = fileread([char(filepath) '/casadi/' char(problem), '.json']);
            end

            % TODO do restart again
            % if ~isempty(bench_name)
            %     outfile = ['~/results/NOSBENCH_results/', BENCH_NAME, '/', char(instance.options.solver_name), '_', char(instance.problem), '.mat'];
            %     if isfile(outfile)
            %         disp(['Skipping: ' outfile])
            %         continue;
            %     end
            % end
            %job.Timeout = 30*60;
            
            task = createTask(job, solver_fun, 1, {json, options, problem});
            task.CaptureDiary = true;
            task.UserData.problem_name = problem;
            task.UserData.options = options;
            
            tasks(ii) = task;
        end
    end
    if delete_problems
        !rm -rf casadi
        !rm -rf vdx
        !rm -rf metadata
    end

    submit(job);
end
